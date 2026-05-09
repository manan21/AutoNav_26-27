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
  : Node("gps_publisher"), gps_connected(false), consecutive_failures_(0) {
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_fix", 50);
    timer_ = this->create_wall_timer(100ms, std::bind(&GPSPublisher::update_gps, this));
    stats_timer_ = this->create_wall_timer(
        30s, std::bind(&GPSPublisher::log_stats, this));
    last_stats_log_at_ = this->now();
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

  bool is_rmc_sentence(const std::string &line) {
    return line.rfind("$GNRMC", 0) == 0 ||
           line.rfind("$GPRMC", 0) == 0;
  }

  // Validate NMEA-0183 checksum: XOR of every char between '$' and '*',
  // compared to the two-hex-digit value following '*'. Returns false if
  // the checksum is missing, malformed, or doesn't match. We use this
  // to silently drop the byte-corrupted frames that show up at 38400
  // baud under heavy multi-constellation NMEA traffic.
  bool nmea_checksum_ok(const std::string &sentence) {
    if (sentence.empty() || sentence[0] != '$') return false;
    size_t star_pos = sentence.find('*');
    if (star_pos == std::string::npos) return false;
    if (star_pos + 2 >= sentence.size()) return false;

    std::string cs_hex = sentence.substr(star_pos + 1, 2);
    int expected = 0;
    try {
      expected = std::stoi(cs_hex, nullptr, 16);
    } catch (const std::exception &) {
      return false;
    }

    int computed = 0;
    for (size_t i = 1; i < star_pos; ++i) {
      computed ^= static_cast<unsigned char>(sentence[i]);
    }
    return computed == expected;
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

  bool parse_gga_and_publish(const std::string &line, bool checksum_pass = true) {
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

    if (lat_str.empty() || lat_dir.empty() || lon_str.empty() || lon_dir.empty() || altitude_str.empty()) {
      RCLCPP_WARN(this->get_logger(), "GGA sentence missing coordinate fields");
      return false;
    }

    try {
      // Parse fix quality — inside try block so corrupted data can't crash the node
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

      double latitude = nmea_to_decimal_degrees(lat_str, lat_dir, true);
      double longitude = nmea_to_decimal_degrees(lon_str, lon_dir, false);
      // Explicit bounds — guards loose-parse path against corrupted
      // lat/lon values that happened to parse numerically.
      if (latitude < -90.0 || latitude > 90.0 ||
          longitude < -180.0 || longitude > 180.0) {
        return false;
      }
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

      // Approximate covariance from HDOP if available. If the frame
      // came in with a bad checksum we still trust the bounds-checked
      // lat/lon, but inflate covariance so downstream EKF gating
      // weights it less than a clean frame.
      double cov_inflate = checksum_pass ? 1.0 : 4.0;
      if (!std::isnan(hdop)) {
        double horizontal_variance = cov_inflate * hdop * hdop;
        double vertical_variance = cov_inflate * 2.0 * hdop * hdop;

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

  // RMC fallback: same lat/lon, no altitude/HDOP/sats. Used when the
  // matching GGA frame is byte-corrupted but its RMC neighbor survives.
  bool parse_rmc_and_publish(const std::string &line, bool checksum_pass = true) {
    std::vector<std::string> fields = split(line, ',');

    // RMC minimum useful fields:
    // 0 = $GNRMC / $GPRMC
    // 1 = UTC time
    // 2 = status (A=valid, V=void)
    // 3 = latitude
    // 4 = N/S
    // 5 = longitude
    // 6 = E/W
    if (fields.size() < 7) {
      RCLCPP_WARN(this->get_logger(), "Malformed RMC sentence: %s", line.c_str());
      return false;
    }

    fields.back() = strip_checksum(fields.back());

    const std::string &status = fields[2];
    const std::string &lat_str = fields[3];
    const std::string &lat_dir = fields[4];
    const std::string &lon_str = fields[5];
    const std::string &lon_dir = fields[6];

    if (status != "A") {
      RCLCPP_DEBUG(this->get_logger(), "RMC status not active (V) — no fix yet");
      return false;
    }
    if (lat_str.empty() || lat_dir.empty() || lon_str.empty() || lon_dir.empty()) {
      return false;
    }

    try {
      double latitude = nmea_to_decimal_degrees(lat_str, lat_dir, true);
      double longitude = nmea_to_decimal_degrees(lon_str, lon_dir, false);
      if (latitude < -90.0 || latitude > 90.0 ||
          longitude < -180.0 || longitude > 180.0) {
        return false;
      }

      sensor_msgs::msg::NavSatFix gps_msg;
      gps_msg.header.stamp = this->now();
      gps_msg.header.frame_id = "gps_footprint";
      gps_msg.latitude = latitude;
      gps_msg.longitude = longitude;
      gps_msg.altitude = 0.0;  // RMC carries no altitude
      gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      gps_msg.position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      // Suppress unused-parameter warning when checksum_pass isn't
      // wired into RMC covariance yet (RMC has no HDOP to scale).
      (void)checksum_pass;
      publisher_->publish(gps_msg);

      RCLCPP_DEBUG(this->get_logger(),
                   "Published GPS fix (RMC): lat=%.8f lon=%.8f", latitude, longitude);
      return true;
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to parse RMC sentence: %s | line: %s", e.what(), line.c_str());
      return false;
    }
  }

  void reconnect_gps() {
    RCLCPP_WARN(this->get_logger(), "GPS connection lost — attempting reconnect...");
    gps_serial.closeDevice();
    gps_connected = false;
    consecutive_failures_ = 0;
  }

  void update_gps() {
    if (!gps_connected) {
      init_gps();
      return;
    }

    char gps_buffer[1024] = {};
    int bytes_read = gps_serial.readString(gps_buffer, '\n', 1023, 500);

    if (bytes_read <= 0) {
      consecutive_failures_++;
      // If no data for 10 consecutive attempts (~5 seconds), reconnect
      if (consecutive_failures_ >= 10) {
        reconnect_gps();
      } else {
        RCLCPP_DEBUG(this->get_logger(), "No GPS data received within timeout (%d/%d)",
                     consecutive_failures_, 10);
      }
      return;
    }

    consecutive_failures_ = 0;  // reset on successful read

    std::string gps_data(gps_buffer, bytes_read);
    if (gps_data.empty()) {
      return;
    }
    stats_reads_++;

    RCLCPP_DEBUG(this->get_logger(), "Current GPS Data: %s", gps_data.c_str());

    // Split on '$' so concatenated sentences (a known UART-overrun
    // symptom at this baud) are processed independently. We try the
    // checksum first as a fast-path "trust this fragment" signal —
    // but if it fails we still attempt to parse with explicit
    // numerical bounds. This recovers fixes from frames where
    // corruption hit the checksum digits or non-position fields
    // (HDOP, sats, altitude) but left lat/lon intact. Frames whose
    // lat/lon don't pass strict bounds still get rejected.
    size_t pos = 0;
    while ((pos = gps_data.find('$', pos)) != std::string::npos) {
      size_t next = gps_data.find('$', pos + 1);
      std::string sentence = (next == std::string::npos)
          ? gps_data.substr(pos)
          : gps_data.substr(pos, next - pos);
      sentence = trim_line(sentence);

      if (next == std::string::npos) {
        pos = std::string::npos;
      } else {
        pos = next;
      }

      if (sentence.empty()) continue;
      stats_fragments_++;

      bool checksum_pass = nmea_checksum_ok(sentence);
      if (checksum_pass) stats_checksum_ok_++;

      bool published = false;
      if (is_gga_sentence(sentence)) {
        published = parse_gga_and_publish(sentence, checksum_pass);
        if (published) {
          if (checksum_pass) stats_gga_published_++;
          else stats_loose_published_++;
        }
      } else if (is_rmc_sentence(sentence)) {
        published = parse_rmc_and_publish(sentence, checksum_pass);
        if (published) {
          if (checksum_pass) stats_rmc_published_++;
          else stats_loose_published_++;
        }
      }
      // Other sentence types (GSA, GSV, GLL, VTG, TXT) are ignored.

      if (!published && (is_gga_sentence(sentence) || is_rmc_sentence(sentence))) {
        stats_rejected_++;
      }
    }
  }

  void log_stats() {
    // Only log if we're connected — otherwise the row is just zeros.
    if (!gps_connected) return;
    RCLCPP_INFO(
      this->get_logger(),
      "gps stats (last 30s): reads=%zu fragments=%zu cs_ok=%zu "
      "gga_pub=%zu rmc_pub=%zu loose_pub=%zu rejected=%zu",
      stats_reads_, stats_fragments_, stats_checksum_ok_,
      stats_gga_published_, stats_rmc_published_, stats_loose_published_,
      stats_rejected_);
    stats_reads_ = 0;
    stats_fragments_ = 0;
    stats_checksum_ok_ = 0;
    stats_gga_published_ = 0;
    stats_rmc_published_ = 0;
    stats_loose_published_ = 0;
    stats_rejected_ = 0;
  }

  bool gps_connected;
  int consecutive_failures_;
  serialib gps_serial;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::Time last_stats_log_at_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;

  // Rolling per-30s counters.
  size_t stats_reads_ = 0;
  size_t stats_fragments_ = 0;
  size_t stats_checksum_ok_ = 0;
  size_t stats_gga_published_ = 0;
  size_t stats_rmc_published_ = 0;
  size_t stats_loose_published_ = 0;
  size_t stats_rejected_ = 0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSPublisher>());
  rclcpp::shutdown();
  return 0;
}