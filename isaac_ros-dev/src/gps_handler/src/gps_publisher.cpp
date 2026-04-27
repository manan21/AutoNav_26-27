#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>

#include "serialib.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

using namespace std::chrono_literals;

// u-blox ZED-F9P serial port and baud rate
#define SERIAL_PORT "/dev/serial/by-id/usb-Cypress_Semiconductor_USB-Serial__Dual_Channel_-if00"
#define SERIAL_BAUD 38400

class GPSPublisher : public rclcpp::Node {
public:
  GPSPublisher()
  : Node("gps_publisher"), gps_connected(false) {
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_fix", 50);
    timer_ = this->create_wall_timer(100ms, std::bind(&GPSPublisher::update_gps, this));
  }

private:
  void init_gps() {
    char opened = gps_serial.openDevice(SERIAL_PORT, SERIAL_BAUD);
    if (opened != 1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %d", SERIAL_PORT, (int)opened);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Connected to u-blox GPS on %s at %d baud", SERIAL_PORT, SERIAL_BAUD);

    // Flush a few startup lines from the serial buffer
    for (int i = 0; i < 10; ++i) {
      char gps_buffer[1024] = {};
      gps_serial.readString(gps_buffer, '\n', 1023, 100);
    }

    gps_connected = true;
  }

  std::vector<std::string> split(const std::string &input, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream ss(input);

    while (std::getline(ss, token, delimiter)) {
      tokens.push_back(token);
    }
    return tokens;
  }

  std::string trim_line(const std::string &line) {
    std::string result = line;

    // Remove trailing newline / carriage return
    while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
      result.pop_back();
    }

    return result;
  }

  std::string strip_checksum(const std::string &field) {
    size_t star_pos = field.find('*');
    if (star_pos != std::string::npos) {
      return field.substr(0, star_pos);
    }
    return field;
  }

  bool is_gga_sentence(const std::string &line) {
    return line.rfind("$GNGGA", 0) == 0 ||
           line.rfind("$GPGGA", 0) == 0;
  }

  double nmea_to_decimal_degrees(const std::string &value, const std::string &direction, bool is_latitude) {
    if (value.empty() || direction.empty()) {
      throw std::runtime_error("Empty NMEA coordinate field");
    }

    // Latitude format: ddmm.mmmmm
    // Longitude format: dddmm.mmmmm
    int degree_digits = is_latitude ? 2 : 3;

    if ((int)value.size() <= degree_digits) {
      throw std::runtime_error("Malformed NMEA coordinate field");
    }

    double degrees = std::stod(value.substr(0, degree_digits));
    double minutes = std::stod(value.substr(degree_digits));
    double decimal = degrees + (minutes / 60.0);

    if (direction == "S" || direction == "W") {
      decimal = -decimal;
    }

    return decimal;
  }

  bool parse_gga_and_publish(const std::string &line) {
    std::vector<std::string> fields = split(line, ',');

    // GGA minimum useful fields:
    // 0 = $GNGGA / $GPGGA
    // 1 = UTC time
    // 2 = latitude
    // 3 = N/S
    // 4 = longitude
    // 5 = E/W
    // 6 = fix quality
    // 7 = number of satellites
    // 8 = HDOP
    // 9 = altitude
    // 10 = altitude units
    // 11 = geoid separation
    // 12 = geoid units
    if (fields.size() < 10) {
      RCLCPP_WARN(this->get_logger(), "Malformed GGA sentence: %s", line.c_str());
      return false;
    }

    // Remove checksum from the last field if present
    fields.back() = strip_checksum(fields.back());

    const std::string &lat_str = fields[2];
    const std::string &lat_dir = fields[3];
    const std::string &lon_str = fields[4];
    const std::string &lon_dir = fields[5];
    const std::string &fix_quality_str = fields[6];
    const std::string &num_sats_str = fields[7];
    const std::string &hdop_str = fields[8];
    const std::string &altitude_str = fields[9];

    if (fix_quality_str.empty()) {
      RCLCPP_WARN(this->get_logger(), "GGA sentence missing fix quality");
      return false;
    }

    int fix_quality = std::stoi(fix_quality_str);

    // u-blox / NMEA fix quality:
    // 0 = invalid
    // 1 = GPS fix
    // 2 = DGPS fix
    // 4 = RTK fixed
    // 5 = RTK float
    if (fix_quality <= 0) {
      RCLCPP_DEBUG(this->get_logger(), "GPS has no valid fix yet");
      return false;
    }

    if (lat_str.empty() || lat_dir.empty() || lon_str.empty() || lon_dir.empty() || altitude_str.empty()) {
      RCLCPP_WARN(this->get_logger(), "GGA sentence missing coordinate fields");
      return false;
    }

    try {
      double latitude = nmea_to_decimal_degrees(lat_str, lat_dir, true);
      double longitude = nmea_to_decimal_degrees(lon_str, lon_dir, false);
      double altitude = std::stod(altitude_str);

      double hdop = std::numeric_limits<double>::quiet_NaN();
      if (!hdop_str.empty()) {
        hdop = std::stod(hdop_str);
      }

      int num_sats = 0;
      if (!num_sats_str.empty()) {
        num_sats = std::stoi(num_sats_str);
      }

      sensor_msgs::msg::NavSatFix gps_msg;
      gps_msg.header.stamp = this->now();
      gps_msg.header.frame_id = "gps_footprint";

      gps_msg.latitude = latitude;
      gps_msg.longitude = longitude;
      gps_msg.altitude = altitude;

      gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

      if (fix_quality > 0) {
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      } else {
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      }

      // Approximate covariance from HDOP if available.
      // This is a rough estimate, but better than always unknown.
      if (!std::isnan(hdop)) {
        double horizontal_variance = hdop * hdop;
        double vertical_variance = 2.0 * hdop * hdop;

        gps_msg.position_covariance = {
          horizontal_variance, 0.0, 0.0,
          0.0, horizontal_variance, 0.0,
          0.0, 0.0, vertical_variance
        };
        gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
      } else {
        gps_msg.position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      }

      publisher_->publish(gps_msg);

      RCLCPP_DEBUG(
        this->get_logger(),
        "Published GPS fix: lat=%.8f lon=%.8f alt=%.2f sats=%d hdop=%.2f fix_quality=%d",
        latitude, longitude, altitude, num_sats, hdop, fix_quality
      );

      return true;
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse GGA sentence: %s | line: %s", e.what(), line.c_str());
      return false;
    }
  }

  void update_gps() {
    if (!gps_connected) {
      init_gps();
      return;
    }

    char gps_buffer[1024] = {};
    int bytes_read = gps_serial.readString(gps_buffer, '\n', 1023, 500);

    if (bytes_read <= 0) {
      RCLCPP_DEBUG(this->get_logger(), "No GPS data received within timeout");
      return;
    }

    std::string gps_data(gps_buffer, bytes_read);
    gps_data = trim_line(gps_data);

    if (gps_data.empty()) {
      return;
    }

    // For debugging
    RCLCPP_DEBUG(this->get_logger(), "Current GPS Data: %s", gps_data.c_str());

    // Only parse GGA for NavSatFix publishing
    if (!is_gga_sentence(gps_data)) {
      return;
    }

    parse_gga_and_publish(gps_data);
  }

  bool gps_connected;
  serialib gps_serial;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSPublisher>());
  rclcpp::shutdown();
  return 0;
}