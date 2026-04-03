#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

// I2C includes
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <functional>

// ====================================================================
// COULOMB COUNTING — commented out until validated on hardware.
// To enable: uncomment every block marked [CC], rebuild, and test.
// ====================================================================

// [CC] Battery constants (Renogy 25 Ah 25.6 V LFP pack, 8s BMS)
// static constexpr double PACK_CAPACITY_MAH  = 25000.0;
// static constexpr int    CELLS_SERIES       = 8;
// static constexpr double CHARGE_TARGET_V    = 29.0;   // charger CV setpoint
// static constexpr double BMS_OV_V           = 29.2;   // BMS over-voltage cutoff
// static constexpr double BMS_UV_V           = 20.0;   // BMS under-voltage cutoff
// static constexpr double TAIL_CURRENT_MA    = 1250.0; // end-of-charge tail
// static constexpr double BMS_OC_MA          = 27500.0;// BMS over-current cutoff
// static constexpr double PEUKERT_EXP        = 1.05;
// static constexpr double PEUKERT_REF_MA     = 5000.0; // C/5 reference rate
// static constexpr double COULOMBIC_EFF      = 0.995;  // charge efficiency

// [CC] LFP per-cell mV → SOC lookup table (17 points)
// Replace with empirical curve fit when available.
// static constexpr struct { double mv; double soc; } LFP_SOC_TABLE[] = {
//     {2500.0,   0.0}, {2800.0,   1.0}, {2900.0,   2.0},
//     {3000.0,   5.0}, {3100.0,   8.0}, {3150.0,  10.0},
//     {3200.0,  20.0}, {3220.0,  30.0}, {3240.0,  40.0},
//     {3250.0,  50.0}, {3260.0,  60.0}, {3270.0,  70.0},
//     {3280.0,  80.0}, {3300.0,  90.0}, {3330.0,  95.0},
//     {3400.0,  99.0}, {3600.0, 100.0},
// };
// static constexpr size_t LFP_SOC_TABLE_LEN =
//     sizeof(LFP_SOC_TABLE) / sizeof(LFP_SOC_TABLE[0]);

// [CC] Charge-state enum
// enum class ChargeState { IDLE, CHARGING_CC, CHARGING_CV, FULL, DISCHARGING };

class ElectricalPublisherNode : public rclcpp::Node {
private:
    // I2C file descriptor
    int i2c_fd_;

    // INA226 conversion factors (per-bit LSB values)
    const double bit_2_mVolt = 1.25;   // 1.25 mV/bit  (bus voltage LSB)
    const double bit_2_mAmp  = 0.25;   // 250 uA/bit   (current LSB)
    const double bit_2_mWatt = 6.25;   // 6.25 mW/bit  (power LSB = 25 * current LSB)

    // INA226 register addresses
    const uint8_t REG_CONFIG  = 0x00;
    const uint8_t REG_VOLTAGE = 0x02;
    const uint8_t REG_POWER   = 0x03;
    const uint8_t REG_CURRENT = 0x04;
    const uint8_t REG_CALIB   = 0x05;
    const uint8_t REG_MFG_ID  = 0xFE;
    const uint8_t REG_DIE_ID  = 0xFF;

    // Latest readings (milli-units)
    double voltage_mV_ = 0.0;
    double current_mA_ = 0.0;
    double power_mW_   = 0.0;

    bool chip_ready_ = false;

    // [CC] Coulomb counter state variables
    // double coulomb_mah_       = 0.0;   // accumulated charge (mAh)
    // rclcpp::Time last_coulomb_time_;    // wall-clock for dt
    // ChargeState charge_state_ = ChargeState::IDLE;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr power_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  heartbeat_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr i2c_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    double loop_rate_hz_      = 10.0;  // I2C polling rate
    double heartbeat_rate_hz_ = 0.1;   // 0.1 Hz = every 10 seconds

    // ================================================================
    // [CC] voltage_to_soc — linear interpolation on LFP per-cell table
    // ================================================================
    // double voltage_to_soc(double pack_mV) {
    //     double cell_mV = pack_mV / CELLS_SERIES;
    //     if (cell_mV <= LFP_SOC_TABLE[0].mv) return 0.0;
    //     if (cell_mV >= LFP_SOC_TABLE[LFP_SOC_TABLE_LEN - 1].mv) return 100.0;
    //     for (size_t i = 1; i < LFP_SOC_TABLE_LEN; ++i) {
    //         if (cell_mV <= LFP_SOC_TABLE[i].mv) {
    //             double t = (cell_mV - LFP_SOC_TABLE[i - 1].mv) /
    //                        (LFP_SOC_TABLE[i].mv - LFP_SOC_TABLE[i - 1].mv);
    //             return LFP_SOC_TABLE[i - 1].soc +
    //                    t * (LFP_SOC_TABLE[i].soc - LFP_SOC_TABLE[i - 1].soc);
    //         }
    //     }
    //     return 100.0;
    // }

    // ================================================================
    // [CC] peukert_capacity — effective capacity at a given discharge rate
    // ================================================================
    // double peukert_capacity(double current_mA) {
    //     if (current_mA <= 0.0) return PACK_CAPACITY_MAH;
    //     double ratio = current_mA / PEUKERT_REF_MA;
    //     return PACK_CAPACITY_MAH * std::pow(ratio, 1.0 - PEUKERT_EXP);
    // }

    // ================================================================
    // I2C polling callback — runs at loop_rate_hz_
    // ================================================================
    void i2c_timer_callback() {
        if (!chip_ready_) {
            return;
        }

        uint8_t buf[2];
        int16_t raw;

        // Voltage
        uint8_t reg = REG_VOLTAGE;
        if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, buf, 2) == 2) {
            raw = (buf[0] << 8) | buf[1];
            voltage_mV_ = raw * bit_2_mVolt;
        }

        // Current
        reg = REG_CURRENT;
        if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, buf, 2) == 2) {
            raw = (buf[0] << 8) | buf[1];
            current_mA_ = raw * bit_2_mAmp;
        }

        // Power
        reg = REG_POWER;
        if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, buf, 2) == 2) {
            raw = (buf[0] << 8) | buf[1];
            power_mW_ = raw * bit_2_mWatt;
        }

        // ============================================================
        // [CC] Coulomb integration (place after current/voltage reads)
        // ============================================================
        // rclcpp::Time now = this->now();
        // if (last_coulomb_time_.nanoseconds() > 0) {
        //     double dt_h = (now - last_coulomb_time_).seconds() / 3600.0;
        //     double abs_mA = std::abs(current_mA_);
        //
        //     if (abs_mA > 50.0) {  // dead-zone: skip < 50 mA
        //         if (current_mA_ > 0.0) {
        //             // Discharging — apply Peukert scaling
        //             double eff_cap = peukert_capacity(abs_mA);
        //             double scale   = PACK_CAPACITY_MAH / eff_cap;
        //             coulomb_mah_  -= abs_mA * dt_h * scale;
        //         } else {
        //             // Charging — apply coulombic efficiency
        //             coulomb_mah_ += abs_mA * dt_h * COULOMBIC_EFF;
        //         }
        //         // Clamp to valid range
        //         if (coulomb_mah_ < 0.0) coulomb_mah_ = 0.0;
        //         if (coulomb_mah_ > PACK_CAPACITY_MAH) coulomb_mah_ = PACK_CAPACITY_MAH;
        //     }
        // }
        // last_coulomb_time_ = now;

        // ============================================================
        // [CC] Charge state machine
        // ============================================================
        // double v = voltage_mV_ / 1000.0;
        // double i = current_mA_;
        // switch (charge_state_) {
        //     case ChargeState::IDLE:
        //         if (i < -TAIL_CURRENT_MA)
        //             charge_state_ = ChargeState::CHARGING_CC;
        //         else if (i > TAIL_CURRENT_MA)
        //             charge_state_ = ChargeState::DISCHARGING;
        //         break;
        //     case ChargeState::CHARGING_CC:
        //         if (v >= CHARGE_TARGET_V)
        //             charge_state_ = ChargeState::CHARGING_CV;
        //         else if (i > -TAIL_CURRENT_MA && i < TAIL_CURRENT_MA)
        //             charge_state_ = ChargeState::IDLE;
        //         break;
        //     case ChargeState::CHARGING_CV:
        //         if (std::abs(i) < TAIL_CURRENT_MA) {
        //             charge_state_ = ChargeState::FULL;
        //             coulomb_mah_  = PACK_CAPACITY_MAH;  // recalibrate
        //         }
        //         break;
        //     case ChargeState::FULL:
        //         if (i > TAIL_CURRENT_MA)
        //             charge_state_ = ChargeState::DISCHARGING;
        //         break;
        //     case ChargeState::DISCHARGING:
        //         if (i < -TAIL_CURRENT_MA)
        //             charge_state_ = ChargeState::CHARGING_CC;
        //         else if (std::abs(i) < TAIL_CURRENT_MA)
        //             charge_state_ = ChargeState::IDLE;
        //         break;
        // }

        // Publish readings (convert milli-units to base units)
        std_msgs::msg::Float32 msg;

        msg.data = voltage_mV_ / 1000.0;
        voltage_pub_->publish(msg);

        msg.data = current_mA_ / 1000.0;
        current_pub_->publish(msg);

        msg.data = power_mW_ / 1000.0;
        power_pub_->publish(msg);
    }

    // ================================================================
    // Heartbeat callback — runs at 0.1 Hz (every 10 seconds)
    // ================================================================
    void heartbeat_callback() {
        double voltage_V = voltage_mV_ / 1000.0;
        double current_A = current_mA_ / 1000.0;
        double power_W   = power_mW_   / 1000.0;

        char buf[128];
        snprintf(buf, sizeof(buf),
                 "V=%.2f V | I=%.3f A | P=%.1f W",
                 voltage_V, current_A, power_W);

        // [CC] Extended heartbeat with SOC:
        // double soc = (coulomb_mah_ / PACK_CAPACITY_MAH) * 100.0;
        // snprintf(buf, sizeof(buf),
        //          "V=%.2f V | I=%.3f A | P=%.1f W | SOC=%.1f%%",
        //          voltage_V, current_A, power_W, soc);

        std_msgs::msg::String heartbeat_msg;
        heartbeat_msg.data = buf;
        heartbeat_pub_->publish(heartbeat_msg);

        RCLCPP_INFO(this->get_logger(), "[Heartbeat] %s", buf);
    }

    // ================================================================
    // I2C helpers
    // ================================================================
    bool open_i2c(const std::string& device, int address) {
        i2c_fd_ = open(device.c_str(), O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device: %s", device.c_str());
            return false;
        }
        if (ioctl(i2c_fd_, I2C_SLAVE, address) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set I2C address: 0x%02X", address);
            close(i2c_fd_);
            return false;
        }
        return true;
    }

    void close_i2c() {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
            i2c_fd_ = -1;
        }
    }

public:
    ElectricalPublisherNode() : Node("electrical_publisher_node"), i2c_fd_(-1) {

        // ============================================================
        // 1. Open I2C bus to INA226 at 0x40
        // ============================================================
        if (!open_i2c("/dev/i2c-7", 0x40)) {
            RCLCPP_ERROR(this->get_logger(), "I2C init failed");
            return;
        }

        // ============================================================
        // 2. Verify MFG ID (0xFE) and DIE ID (0xFF)
        // ============================================================
        uint8_t reg;
        uint8_t read_buf[2];

        // MFG ID — expect 0x5449 ("TI")
        reg = REG_MFG_ID;
        if (write(i2c_fd_, &reg, 1) != 1 || read(i2c_fd_, read_buf, 2) != 2 ||
            read_buf[0] != 0x54 || read_buf[1] != 0x49) {
            RCLCPP_ERROR(this->get_logger(),
                "INA226 MFG ID mismatch (got 0x%02X%02X, expected 0x5449)",
                read_buf[0], read_buf[1]);
            return;
        }

        // DIE ID — expect 0x2260 (INA226)
        reg = REG_DIE_ID;
        if (write(i2c_fd_, &reg, 1) != 1 || read(i2c_fd_, read_buf, 2) != 2 ||
            read_buf[0] != 0x22 || read_buf[1] != 0x60) {
            RCLCPP_ERROR(this->get_logger(),
                "INA226 DIE ID mismatch (got 0x%02X%02X, expected 0x2260)",
                read_buf[0], read_buf[1]);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "INA226 identified (MFG=TI, DIE=0x2260)");

        // ============================================================
        // 3. Write configuration register (0x00) = 0x4207
        //    AVG=4, VBUSCT=140us, VSHCT=140us, continuous shunt+bus
        // ============================================================
        uint8_t config_buf[3] = {REG_CONFIG, 0x42, 0x07};
        if (write(i2c_fd_, config_buf, sizeof(config_buf)) != sizeof(config_buf)) {
            RCLCPP_ERROR(this->get_logger(), "Config register write failed");
            return;
        }

        // ============================================================
        // 4. Write calibration register (0x05) = 0x06AA  (CAL = 1706)
        //    CAL = 0.00512 / (250uA * 0.012 ohm) = 1706
        //    Shunt resistor R4 = 12 mOhm
        // ============================================================
        uint8_t calib_buf[3] = {REG_CALIB, 0x06, 0xAA};
        if (write(i2c_fd_, calib_buf, sizeof(calib_buf)) != sizeof(calib_buf)) {
            RCLCPP_ERROR(this->get_logger(), "Calibration write failed");
            return;
        }

        // Read back calibration register to verify
        reg = REG_CALIB;
        if (write(i2c_fd_, &reg, 1) == 1 &&
            read(i2c_fd_, read_buf, 2) == 2 &&
            read_buf[0] == 0x06 &&
            read_buf[1] == 0xAA) {

            chip_ready_ = true;
            RCLCPP_INFO(this->get_logger(),
                "INA226 calibrated and ready (CAL=1706, R_shunt=12mOhm)");
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "INA226 calibration readback failed (got 0x%02X%02X, expected 0x06AA)",
                read_buf[0], read_buf[1]);
        }

        // [CC] Initialize coulomb counter from voltage-based SOC estimate
        // coulomb_mah_       = voltage_to_soc(voltage_mV_) / 100.0 * PACK_CAPACITY_MAH;
        // last_coulomb_time_ = this->now();
        // charge_state_      = ChargeState::IDLE;

        // ============================================================
        // 5. Create publishers
        // ============================================================
        voltage_pub_   = this->create_publisher<std_msgs::msg::Float32>("/electrical/voltage", 10);
        current_pub_   = this->create_publisher<std_msgs::msg::Float32>("/electrical/current", 10);
        power_pub_     = this->create_publisher<std_msgs::msg::Float32>("/electrical/power", 10);
        heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>("/electrical/heartbeat", 10);

        // ============================================================
        // 6. Create timers
        // ============================================================
        auto heartbeat_period = std::chrono::duration<double>(1.0 / heartbeat_rate_hz_);
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(heartbeat_period),
            std::bind(&ElectricalPublisherNode::heartbeat_callback, this));

        auto timer_period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
        i2c_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
            std::bind(&ElectricalPublisherNode::i2c_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "I2C polling loop started at %.1f Hz", loop_rate_hz_);
    }

    ~ElectricalPublisherNode() {
        close_i2c();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ElectricalPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
