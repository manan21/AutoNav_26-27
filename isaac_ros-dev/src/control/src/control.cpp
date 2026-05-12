#include <rclcpp/rclcpp.hpp>
#include "serialib.hpp"
#include "xbox.hpp"
#include "motor_controller.hpp"
#include "autonomous.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autonav_interfaces/msg/encoders.hpp"
#include "autonav_interfaces/srv/configure_control.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

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

        if(!autonomousMode){
            controller.set_b(joy_msg->buttons[1]);
            controller.set_x(joy_msg->buttons[3]);
            controller.set_y(joy_msg->buttons[2]);

            // Bumpers are not in a stable position across the controllers
            // / xpad-driver combinations we've seen on this Jetson: live
            // /joy diagnostics on 2026-05-10 captured presses at indices
            // {6, 7} in one session and {9, 10} in another. Rather than
            // pin to one layout and silently break on the next swap, OR
            // both index pairs. Trade-off: on a layout where 9/10 are
            // stick clicks instead of bumpers, clicking a stick will
            // also nudge the speed — non-critical UX cost vs. the
            // bumpers becoming inert.
            auto safe_btn = [&joy_msg](size_t i) -> int {
                return i < joy_msg->buttons.size() ? joy_msg->buttons[i] : 0;
            };
            controller.set_right_bumper(safe_btn(6) || safe_btn(10));
            controller.set_left_bumper(safe_btn(7) || safe_btn(9));

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
                motors.move(command.right_motor_speed * motors.getSpeed(), command.left_motor_speed * motors.getSpeed());
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
           motors.move(right_wheel_speed * 40, left_wheel_speed * 40);

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

