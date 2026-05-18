#include <rclcpp/rclcpp.hpp>
#include "serialib.hpp"
#include "xbox.hpp"
#include "motor_controller.hpp"
#include "autonomous.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autonav_interfaces/msg/encoders.hpp"
#include "autonav_interfaces/srv/configure_control.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <chrono>

#define WHEEL_BASE 0.6858

#define DEBUG_ESTOP

class ControlNode : public rclcpp::Node {

    public:


    ControlNode() 
      : Node("control_node")
       {

        // topic names
        this->declare_parameter("controller_topic", "joy");
        this->declare_parameter("encoder_topic", "encoders");
        this->declare_parameter("path_planning_topic", "cmd_vel");

        // serial ports test
        this->declare_parameter(
            "motor_port",
            "/dev/serial/by-id/usb-RoboteQ_RoboteQ_FBLG2360T_HABJAA5QR0E5NDEg_207A34554147-if00");

        this->declare_parameter(
            "arduino_port",
            "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_8583030363935190F141-if00");

        this->declare_parameter("estop_port", "/dev/ttyTHS1");

        // PHASE D — grade compensation (gravity-vector approach). Reads
        // the IMU's body-frame X accelerometer reading (which IS the
        // gravity projection along the robot's longitudinal axis at
        // quasi-static velocities), low-pass filters it, and multiplies
        // the motor command by a clamped multiplier so the 20 lb
        // payload doesn't stall on a 15 % grade going up, and the
        // robot doesn't run away going down. Disabled by default — set
        // `grade_comp_enabled: true` (or `ros2 param set ...`) to turn
        // it on. First test should be on a tilt block with this flag
        // OFF, watch a_fwd in the log to confirm sign and magnitude,
        // then enable.
        this->declare_parameter("imu_topic", "/sick_scansegment_xd/imu_inflated");
        // SICK is upside-down on this robot (per imu_cov_inflator
        // OSCILLATION-SENSITIVE comments). Sign = -1.0 flips the body-X
        // accel reading so positive = nose-up = forward-opposing gravity.
        // If you switch to /zed/zed_node/imu/data (right-side-up), set
        // this param to +1.0.
        this->declare_parameter("imu_a_fwd_sign", -1.0);
        this->declare_parameter("grade_comp_enabled", false);
        // Bounded linear map from pitch (deg) to multiplier delta.
        // Mathematically incapable of runaway: input clamped to
        // [-max_deg, +max_deg], output clamped to
        // [1 - max_downhill_pct, 1 + max_uphill_pct].
        //
        //   pitch ≤ -max_deg   →   multiplier = 1 - max_downhill_pct
        //   pitch =  0         →   multiplier = 1.0
        //   pitch ≥ +max_deg   →   multiplier = 1 + max_uphill_pct
        //   linear in between
        //
        // Deadband is small (±0.5°) — just filters IMU noise at level.
        // Pitch is reconstructed from a_fwd via asin(a_fwd/g), with
        // the input clamped so a transient |a_fwd| > g (from real
        // forward acceleration) cannot produce NaN.
        this->declare_parameter("grade_comp_max_deg", 10.0);
        this->declare_parameter("grade_comp_max_uphill_pct", 0.10);
        this->declare_parameter("grade_comp_max_downhill_pct", 0.10);
        this->declare_parameter("grade_comp_deadband_deg", 0.5);
        // If no IMU message for this many seconds, multiplier reverts
        // to 1.0 — prevents stale-gravity compensation if the IMU
        // pipeline silently dies mid-mission.
        this->declare_parameter("grade_comp_imu_timeout_sec", 0.5);
        // EWMA alpha on a_fwd. 0.2 = ~5-sample decay at 50 Hz IMU,
        // smooths out wheel-driven body acceleration spikes so we
        // estimate the slow-changing gravity component cleanly.
        this->declare_parameter("grade_comp_alpha", 0.2);

        configure_server = this->create_service<autonav_interfaces::srv::ConfigureControl>
             ("configure_control", std::bind(&ControlNode::configure, this, std::placeholders::_1, std::placeholders::_2));

    }

    serialib arduinoSerial;
    Xbox controller;
    MotorController motors;
    Autonomous currPose;

    private:
    bool autonomousMode = false;
    float linear_move;
    float angular_move;
    float left_wheel_speed;
    float right_wheel_speed;

    // subscription for joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controllerSub;
    //configuration server
    rclcpp::Service<autonav_interfaces::srv::ConfigureControl>::SharedPtr configure_server;

    // publisher for encoder values
     rclcpp::Publisher<autonav_interfaces::msg::Encoders>::SharedPtr encodersPub;
     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autonomous_mode_pub_;
     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr estop_sub_;
     rclcpp::TimerBase::SharedPtr encoder_timer_;

    // rclcpp::TimerBase::SharedPtr joy_timer_;
    // subscription for Nav2 pose
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pathPlanningSub;

    // Speed publisher for DAQ logging
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speed_pub_;

    // Debounce for bumper speed changes (500ms between changes)
    std::chrono::steady_clock::time_point last_speed_change_time_{};

    // /joy watchdog: timestamp of last received /joy. Initialized to
    // epoch so the watchdog also fires before the first /joy arrives.
    rclcpp::Time last_joy_msg_time_{0, 0, RCL_ROS_TIME};

    bool currX = false;
    bool prevX = false;

    // Y button rising-edge -> clear global costmap. Testing aid during
    // mission iteration when accumulated obstacle cells block planning.
    // Index 2 matches the existing controller.set_y(joy_msg->buttons[2])
    // mapping below. If a wrong container mount swaps X and Y, swap this
    // index with the X autonomous-toggle index in joystick_callback.
    bool currY_clear = false;
    bool prevY_clear = false;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_client_;

    // PHASE D — gravity-vector grade compensation state.
    double latest_a_fwd_ = 0.0;             // EWMA-filtered body-X accel.
    bool have_imu_ = false;
    rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;

    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        last_joy_msg_time_ = this->now();
        currX = joy_msg->buttons[3];

        // Detect rising edge
        if (!prevX && currX) {
            autonomousMode = !autonomousMode;

            if (autonomousMode) {
                char mode[12] = "AUTONOMOUS\n";
                arduinoSerial.writeString(mode);
            } else {
                char mode[8] = "MANUAL\n";
                arduinoSerial.writeString(mode);
            }

            // Publish mode so NAV2 / GPS waypoint handler know the state
            if (autonomous_mode_pub_) {
                std_msgs::msg::Bool mode_msg;
                mode_msg.data = autonomousMode;
                autonomous_mode_pub_->publish(mode_msg);
            }
        }

        prevX = currX;

        // Y rising-edge -> request global costmap clear. Fires in both
        // AUTO and MANUAL modes since the costmap state is shared.
        // No service_is_ready() gate — rclcpp queues async_send_request
        // regardless; we want the press to register even if the server
        // is briefly busy clearing+rebuilding from a previous call.
        int curr_y_btn = (joy_msg->buttons.size() > 2) ? joy_msg->buttons[2] : 0;
        currY_clear = curr_y_btn != 0;
        if (!prevY_clear && currY_clear && clear_global_costmap_client_) {
            auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
            clear_global_costmap_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(),
                "Y pressed -> /global_costmap/clear_entirely_global_costmap");
        }
        prevY_clear = currY_clear;

        if(!autonomousMode){
            controller.set_b(joy_msg->buttons[1]);
            controller.set_x(joy_msg->buttons[3]);
            controller.set_y(joy_msg->buttons[2]);

            // Bumpers are not in a stable position across the controllers
            // / xpad-driver combinations we've seen on this Jetson: live
            // /joy diagnostics captured presses at indices {6, 7} in the
            // normal mount and {9, 10} in the wrong-container-mount
            // layout. The mapping pairs right=6/9 and left=7/10. Rather
            // than pin to one layout and silently break on the next
            // swap, OR both index pairs. Trade-off: on a layout where
            // 9/10 are stick clicks instead of bumpers, clicking a
            // stick will also nudge the speed — non-critical UX cost
            // vs. the bumpers becoming inert.
            auto safe_btn = [&joy_msg](size_t i) -> int {
                return i < joy_msg->buttons.size() ? joy_msg->buttons[i] : 0;
            };
            controller.set_right_bumper(safe_btn(6) || safe_btn(9));
            controller.set_left_bumper(safe_btn(7) || safe_btn(10));

            controller.set_left_stick_x(joy_msg->axes[0]);
            controller.set_left_stick_y(joy_msg->axes[1]);
            controller.set_right_stick_x(joy_msg->axes[2]);
            controller.set_right_stick_y(joy_msg->axes[3]);

        }
    }

    void estop_callback(const std_msgs::msg::String::SharedPtr msg){


	   
            
	    std::string incoming = msg->data;

            
            if (incoming.empty()) {
	       #ifdef DEBUG_ESTOP
	       RCLCPP_INFO(this->get_logger(), "incoming string empty");
	       #endif

                return;

            }

            #ifdef DEBUG_ESTOP
            RCLCPP_INFO(this->get_logger(), "incoming string: %s", incoming.c_str());
            #endif

            if (incoming == "STOP") {
                RCLCPP_WARN(this->get_logger(), "ESTOP PRESSED: MOTORS SHUTTING DOWN");
                motors.shutdown();
            }

    }

    // PHASE D — gravity-vector callback. Body-frame X accel reading
    // contains both linear acceleration and the projection of gravity
    // along base_link's forward axis. At quasi-static velocities (or
    // when filtered) the gravity projection dominates: a_fwd = g*sin(pitch)
    // for a nose-up slope. EWMA-filtered here so wheel-driven acceleration
    // spikes don't contaminate the grade estimate.
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_stamp_ = this->now();
        const double sign = this->get_parameter("imu_a_fwd_sign").as_double();
        const double alpha = this->get_parameter("grade_comp_alpha").as_double();
        const double raw_ax = sign * msg->linear_acceleration.x;
        if (!have_imu_) {
            latest_a_fwd_ = raw_ax;
            have_imu_ = true;
        } else {
            latest_a_fwd_ = alpha * raw_ax + (1.0 - alpha) * latest_a_fwd_;
        }
    }

    // PHASE D — bounded linear-map multiplier. Returns 1.0 when:
    //   - feature disabled via param
    //   - IMU silent (timeout)
    //   - forward_command <= 0 (don't boost rotations or BackUp recovery)
    //   - a_fwd is not finite (NaN/inf guard)
    //   - |pitch| inside deadband (level ground)
    //
    // Otherwise, linear pitch-to-delta mapping, clamped at ±max_deg.
    // The clamps make runaway impossible by construction:
    //   pitch (clamped to ±max_deg) / max_deg gives t in [-1, +1]
    //   delta = max_uphill_pct * t  (if t ≥ 0)
    //         = max_downhill_pct * t  (if t < 0, so delta is negative)
    //   multiplier = 1 + delta
    // So with defaults (max_uphill_pct = max_downhill_pct = 0.10),
    // multiplier ∈ [0.90, 1.10] REGARDLESS of IMU input.
    double grade_speed_multiplier(double forward_command) {
        if (!this->get_parameter("grade_comp_enabled").as_bool()) return 1.0;
        if (!have_imu_) return 1.0;
        const double timeout = this->get_parameter("grade_comp_imu_timeout_sec").as_double();
        if ((this->now() - last_imu_stamp_).seconds() > timeout) return 1.0;
        if (forward_command <= 0.0) return 1.0;
        if (!std::isfinite(latest_a_fwd_)) return 1.0;

        // Reconstruct pitch from gravity projection. Clamp the
        // a_fwd/g ratio to [-1, 1] BEFORE asin so a transient
        // |a_fwd| > g (real forward acceleration spike, e.g. during
        // a hard start) cannot return NaN.
        constexpr double g = 9.81;
        const double sin_pitch = std::clamp(latest_a_fwd_ / g, -1.0, 1.0);
        const double pitch_deg = std::asin(sin_pitch) * 180.0 / M_PI;

        const double deadband_deg =
            this->get_parameter("grade_comp_deadband_deg").as_double();
        if (std::abs(pitch_deg) < deadband_deg) return 1.0;

        const double max_deg = this->get_parameter("grade_comp_max_deg").as_double();
        const double max_up_pct =
            this->get_parameter("grade_comp_max_uphill_pct").as_double();
        const double max_dn_pct =
            this->get_parameter("grade_comp_max_downhill_pct").as_double();

        const double t = std::clamp(pitch_deg / max_deg, -1.0, 1.0);
        const double delta = (t >= 0.0) ? (max_up_pct * t) : (max_dn_pct * t);
        return 1.0 + delta;
    }


    void publish_encoder_data() {
        autonav_interfaces::msg::Encoders encoder_msg;
        encoder_msg.left_motor_rpm = 0;
        encoder_msg.right_motor_rpm = 0;
        encoder_msg.left_motor_count = motors.getLeftEncoderCount();
        encoder_msg.right_motor_count = motors.getRightEncoderCount();
        //RCLCPP_INFO(this->get_logger(), "LEC: %s", motors.getLeftEncoderCount());


        std::string arduinoEncoderCounts = "L:";
        arduinoEncoderCounts += encoder_msg.left_motor_count;
        arduinoEncoderCounts += " R:";
        arduinoEncoderCounts += encoder_msg.right_motor_count;



        if(!autonomousMode){
            // Joy watchdog: 0.5 s without /joy → zero motors.
            if ((this->now() - last_joy_msg_time_).seconds() > 0.5) {
                motors.move(0, 0);
            } else {
            Xbox::CommandData command = controller.calculateCommand();

            if(command.cmd == Xbox::MOVE){
                // PHASE D — also apply grade compensation in manual mode
                // so the multiplier can be calibrated on a tilt block
                // before relying on it in autonomous. Same forward-only
                // gate as the autonomous path.
                const double forward_cmd_manual = 0.5 * (
                    command.left_motor_speed + command.right_motor_speed);
                const double mult_manual = grade_speed_multiplier(forward_cmd_manual);
                motors.move(
                    command.right_motor_speed * motors.getSpeed() * mult_manual,
                    command.left_motor_speed * motors.getSpeed() * mult_manual);
            }
            else if(command.cmd == Xbox::SPEED_DOWN){
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_speed_change_time_).count();
                if (elapsed >= 200) {
                    motors.setSpeed(motors.getSpeed() - 1);
                    last_speed_change_time_ = now;
                    RCLCPP_INFO(this->get_logger(), "speed down. new speed: %d", motors.getSpeed());
                }
            }
            else if(command.cmd == Xbox::SPEED_UP){
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_speed_change_time_).count();
                if (elapsed >= 200) {
                    motors.setSpeed(motors.getSpeed() + 1);
                    last_speed_change_time_ = now;
                    RCLCPP_INFO(this->get_logger(), "speed up! new speed: %d", motors.getSpeed());
                }
            }
            else if(command.cmd == Xbox::STOP){
                motors.shutdown();
            }
            }  // end joy-watchdog else
        }
        else {
           // PHASE D — apply gravity-vector grade compensation. Forward
           // command = average of left/right wheel speeds (positive means
           // forward intent). The multiplier is 1.0 unless climbing or
           // descending past the deadband; clamped to safe motor-current
           // bounds by grade_comp_max_uphill_multiplier /
           // grade_comp_min_downhill_multiplier.
           const double forward_cmd = 0.5 * (left_wheel_speed + right_wheel_speed);
           const double mult = grade_speed_multiplier(forward_cmd);
           motors.move(right_wheel_speed * 40 * mult, left_wheel_speed * 40 * mult);

        }

        arduinoEncoderCounts += "\n";
        // arduinoSerial.writeString(arduinoEncoderCounts.c_str());

        if (encodersPub) {
            encodersPub->publish(encoder_msg);
        }

        // Publish current speed for DAQ logging
        if (speed_pub_) {
            auto speed_msg = std_msgs::msg::String();
            speed_msg.data = std::to_string(motors.getSpeed());
            speed_pub_->publish(speed_msg);
        }
    }

    void path_planning_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

        if (autonomousMode) {
            /*Move to Pose Way*/
            // currPose.linearX = msg->linear.x;
            // currPose.linearY = msg->linear.y;
            //currPose.linearZ = msg->linear.z;

            // currPose.angularZ = msg->angular.z;

            // currPose.goToPose(linearX, linearY, motors);

            /*Direct Speed to Motor Way*/

            linear_move = msg->linear.x;
            angular_move = msg->angular.z;

            left_wheel_speed = linear_move - ( angular_move * (WHEEL_BASE/2));
            right_wheel_speed = linear_move +( angular_move * (WHEEL_BASE/2));

        }
    }


    void init_serial_arduino(const char * arduino_port) {

        char ret;
        ret = arduinoSerial.openDevice(arduino_port, 9600);

        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Arduino serial error: %s", arduinoSerial.error_map.at(ret).c_str());

        }
	else {
	    RCLCPP_INFO(this->get_logger(), "arduino connection success!");
	}


        char mode[8] = "MANUAL\n";
        arduinoSerial.writeString(mode);
    }



    void configure(const std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Request> request, 
                         std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Response> response) {


        // configure serial
        std::string motor_port  = this->get_parameter("motor_port").as_string();
        std::string arduino_port = this->get_parameter("arduino_port").as_string();
        std::string estop_port = this->get_parameter("estop_port").as_string();


        if (request->arduino) {
            init_serial_arduino(arduino_port.c_str());
        }
        if (request->motors) {
            char ret = motors.configure(motor_port.c_str());
            if (ret != 1) {
                // RCLCPP_ERROR(this->get_logger(), "Motor serial error: %s", .error_map.at(ret).c_str());
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Motor serial connection success!");
            }

        }

	    
        estop_sub_ = this->create_subscription<std_msgs::msg::String>("/estop", 10, std::bind(&ControlNode::estop_callback, this, std::placeholders::_1));


        std::string leftMotorCommand = "!C 1 0\r";
        std::string rightMotorCommand = "!C 2 0 \r";
        motors.motorSerial.writeString(leftMotorCommand.c_str());
        motors.motorSerial.writeString(rightMotorCommand.c_str());

        // configure topics
        std::string controller_topic = this->get_parameter("controller_topic").as_string();
        std::string encoder_topic = this->get_parameter("encoder_topic").as_string();
        std::string path_planning_topic = "cmd_vel";

        //XBOX SUB
        controllerSub = this->create_subscription<sensor_msgs::msg::Joy>(
            controller_topic, 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));

        // Client backing the Y-button shortcut that clears the global
        // costmap during testing. Service is hosted by nav2_costmap_2d
        // on the global_costmap node.
        clear_global_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
            "/global_costmap/clear_entirely_global_costmap");

        // PHASE D — IMU subscription for gravity-vector grade compensation.
        // imu_topic param defaults to /sick_scansegment_xd/imu_inflated
        // (already on the bus for ekf_local; SICK is upside-down so the
        // sign flip lives in imu_a_fwd_sign). Switch to /zed/zed_node/imu/data
        // if the SICK path is unavailable; set imu_a_fwd_sign to +1.0
        // for the ZED side.
        const std::string imu_topic =
            this->get_parameter("imu_topic").as_string();
        imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&ControlNode::imu_callback, this, std::placeholders::_1));

        // ESTOP CALLBACK
	
        
        // Publish autonomous mode state so NAV2 / GPS waypoint handler can react
        autonomous_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/autonomous_mode", 10);
        {
            std_msgs::msg::Bool mode_msg;
            mode_msg.data = autonomousMode;
            autonomous_mode_pub_->publish(mode_msg);
        }

        // ----- HARD GUARD FOR ENCODER PUBLISHER -----
        auto existing_pubs = this->get_publishers_info_by_topic(encoder_topic, false);

        if (!existing_pubs.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Another node already publishes %s. Skipping encoder publisher, but motor control timer still active.",
                encoder_topic.c_str());
        } else {
            // NAVIGATION ENCODER PUB
            encodersPub = this->create_publisher<autonav_interfaces::msg::Encoders>(encoder_topic, 10);
        }

        // Always create the control timer — motor commands and speed changes depend on it
        encoder_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ControlNode::publish_encoder_data, this));
        // ----- END HARD GUARD -----

        // Speed publisher for DAQ logging
        speed_pub_ = this->create_publisher<std_msgs::msg::String>("/motor_speed", 10);

       /* joy_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControlNode::joy_timer_callback, this)
        );*/
        //GPS PUB
        //PATH PLANNING SUB
        pathPlanningSub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ControlNode::path_planning_callback, this, std::placeholders::_1));


        response->ret = 0;
    }
};


int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;

}

