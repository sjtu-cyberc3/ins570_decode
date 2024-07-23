#include "ins570_decode/ins570_decode.hpp"

// ros
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// boost
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>

// std
#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace ins570_decode {

struct DecodeNode::Impl {
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>> vel_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> fix_pub;

    std::atomic_bool req_stop = false;
    std::thread rx_thread;

    int leap_seconds;
    std::string rs422_port;
    std::string frame_id;
};

DecodeNode::DecodeNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("ins570_decode_node", options), pImpl(std::make_unique<Impl>()) {
#define PARAM(var, ...)                                               \
    [this]() {                                                        \
        declare_parameter<decltype(pImpl->var)>(#var, ##__VA_ARGS__); \
        get_parameter(#var, pImpl->var);                              \
        RCLCPP_INFO_STREAM(get_logger(), #var ": " << pImpl->var);    \
    }()
    PARAM(leap_seconds, 18);
    PARAM(rs422_port);
    PARAM(frame_id, "inertial");
#undef PARAM

    pImpl->vel_pub = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("vel", 1);
    pImpl->imu_pub = create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    pImpl->fix_pub = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);

    pImpl->rx_thread = std::thread([this]() {
        boost::asio::io_context ioctx;
        boost::asio::serial_port port{ioctx};
        std::vector<uint8_t> data(63);
        port.open(pImpl->rs422_port);
        port.set_option(boost::asio::serial_port::baud_rate(230400));

        double position_std[3] = {0};
        double velocity_std[3] = {0};
        double orientation_std[3] = {0};
        int8_t gps_status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

        while (!pImpl->req_stop) {
            // read frame
            try {
                // read frame header
                boost::asio::read(port, boost::asio::buffer(&data[0], 1));
                if (data[0] != 0xBD) continue;
                boost::asio::read(port, boost::asio::buffer(&data[1], 1));
                if (data[1] != 0xDB) continue;
                boost::asio::read(port, boost::asio::buffer(&data[2], 1));
                if (data[2] != 0x0B) continue;
                // read frame body
                boost::asio::read(port, boost::asio::buffer(&data[3], 60));
            } catch (const boost::system::system_error &err) {
                RCLCPP_ERROR(get_logger(), "Error on read: %s, tring to reopen port...", err.what());
                port.close();
                std::this_thread::sleep_for(1s);
                port.open(pImpl->rs422_port);
                port.set_option(boost::asio::serial_port::baud_rate(230400));
                continue;
            }
            // checksum
            uint8_t bcc0 = 0xBD ^ 0xDB ^ 0x0B;
            for (int i = 3; i <= 56; i++) {
                bcc0 ^= data[i];
            }
            if (bcc0 != data[57]) {
                RCLCPP_WARN(get_logger(), "checksum bcc0 fail, skip frame!");
                continue;
            }
            uint8_t bcc1 = bcc0;
            for (int i = 57; i <= 61; i++) {
                bcc1 ^= data[i];
            }
            if (bcc1 != data[62]) {
                RCLCPP_WARN(get_logger(), "checksum bcc1 fail, skip frame!");
                continue;
            }
            // decode frame
            double roll = *reinterpret_cast<int16_t *>(&data[3]) * 360. / 32768. / 180. * M_PI;
            double pitch = *reinterpret_cast<int16_t *>(&data[5]) * 360. / 32768. / 180. * M_PI;
            double yaw = *reinterpret_cast<int16_t *>(&data[7]) * 360. / 32768. / 180. * M_PI;
            double cy = std::cos(yaw * 0.5);
            double sy = std::sin(yaw * 0.5);
            double cp = std::cos(pitch * 0.5);
            double sp = std::sin(pitch * 0.5);
            double cr = std::cos(roll * 0.5);
            double sr = std::sin(roll * 0.5);
            double qw = cy * cp * cr + sy * sp * sr;
            double qx = cy * cp * sr - sy * sp * cr;
            double qy = sy * cp * sr + cy * sp * cr;
            double qz = sy * cp * cr - cy * sp * sr;
            double gyro_x = *reinterpret_cast<int16_t *>(&data[9]) * 300. / 32768. / 180. * M_PI;
            double gyro_y = *reinterpret_cast<int16_t *>(&data[11]) * 300. / 32768. / 180. * M_PI;
            double gyro_z = *reinterpret_cast<int16_t *>(&data[13]) * 300. / 32768. / 180. * M_PI;
            double acc_x = *reinterpret_cast<int16_t *>(&data[15]) * 12. / 32768.;
            double acc_y = *reinterpret_cast<int16_t *>(&data[17]) * 12. / 32768.;
            double acc_z = *reinterpret_cast<int16_t *>(&data[19]) * 12. / 32768.;
            double lat = *reinterpret_cast<int32_t *>(&data[21]) * 1e-7;
            double lon = *reinterpret_cast<int32_t *>(&data[25]) * 1e-7;
            double alt = *reinterpret_cast<int32_t *>(&data[29]) * 1e-3;
            double vel_n = *reinterpret_cast<int16_t *>(&data[33]) * 1e2 / 32768.;
            double vel_e = *reinterpret_cast<int16_t *>(&data[35]) * 1e2 / 32768.;
            double vel_d = *reinterpret_cast<int16_t *>(&data[37]) * 1e2 / 32768.;
            [[maybe_unused]] uint8_t init_status = *reinterpret_cast<uint8_t *>(&data[39]);
            int16_t data1 = *reinterpret_cast<int16_t *>(&data[46]);
            int16_t data2 = *reinterpret_cast<int16_t *>(&data[48]);
            int16_t data3 = *reinterpret_cast<int16_t *>(&data[50]);
            uint8_t datatype = *reinterpret_cast<uint8_t *>(&data[56]);
            switch (datatype) {
                case 0:  // position std
                    position_std[0] = std::exp(data1 / 100.);
                    position_std[1] = std::exp(data2 / 100.);
                    position_std[2] = std::exp(data3 / 100.);
                    break;
                case 1:  // velocity std
                    velocity_std[0] = std::exp(data1 / 100.);
                    velocity_std[1] = std::exp(data2 / 100.);
                    velocity_std[2] = std::exp(data3 / 100.);
                    break;
                case 2:  // orientation std
                    orientation_std[0] = std::exp(data1 / 100.) / 180. * M_PI;
                    orientation_std[1] = std::exp(data2 / 100.) / 180. * M_PI;
                    orientation_std[2] = std::exp(data3 / 100.) / 180. * M_PI;
                    break;
                case 32:  // gps status
                    if ((48 <= data1 && data1 <= 50) && (48 <= data3 && data3 <= 50)) {
                        gps_status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    } else if ((32 <= data1 && data1 <= 34) || (32 <= data3 && data3 <= 34)) {
                        gps_status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                    } else {
                        gps_status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    }
                    break;
            }
            uint64_t gps_ns = *reinterpret_cast<uint32_t *>(&data[52]) * 250000ull;
            uint64_t gps_week = *reinterpret_cast<uint32_t *>(&data[58]);
            uint64_t utc_sec = 315964800ull + gps_week * 7 * 24 * 3600 + gps_ns / 1000000000ull - pImpl->leap_seconds;
            uint64_t utc_ns = gps_ns % 1000000000;

            // publish msg
            {
                auto p_vel_msg = std::make_unique<geometry_msgs::msg::TwistWithCovarianceStamped>();
                p_vel_msg->header.frame_id = pImpl->frame_id;
                p_vel_msg->header.stamp.sec = utc_sec;
                p_vel_msg->header.stamp.nanosec = utc_ns;
                p_vel_msg->twist.twist.linear.x = vel_n;
                p_vel_msg->twist.twist.linear.y = vel_e;
                p_vel_msg->twist.twist.linear.z = vel_d;
                p_vel_msg->twist.covariance[0] = velocity_std[0] * velocity_std[0];
                p_vel_msg->twist.covariance[7] = velocity_std[1] * velocity_std[1];
                p_vel_msg->twist.covariance[14] = velocity_std[2] * velocity_std[2];
                pImpl->vel_pub->publish(std::move(p_vel_msg));
            }
            {
                auto p_imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
                p_imu_msg->header.frame_id = pImpl->frame_id;
                p_imu_msg->header.stamp.sec = utc_sec;
                p_imu_msg->header.stamp.nanosec = utc_ns;
                p_imu_msg->angular_velocity.x = gyro_x;
                p_imu_msg->angular_velocity.y = gyro_y;
                p_imu_msg->angular_velocity.z = gyro_z;
                p_imu_msg->linear_acceleration.x = acc_x;
                p_imu_msg->linear_acceleration.y = acc_y;
                p_imu_msg->linear_acceleration.z = acc_z;
                p_imu_msg->orientation.w = qw;
                p_imu_msg->orientation.x = qx;
                p_imu_msg->orientation.y = qy;
                p_imu_msg->orientation.z = qz;
                p_imu_msg->orientation_covariance[0] = orientation_std[0] * orientation_std[0];
                p_imu_msg->orientation_covariance[4] = orientation_std[1] * orientation_std[1];
                p_imu_msg->orientation_covariance[8] = orientation_std[2] * orientation_std[2];
                pImpl->imu_pub->publish(std::move(p_imu_msg));
            }
            {
                auto p_fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
                p_fix_msg->header.frame_id = pImpl->frame_id;
                p_fix_msg->header.stamp.sec = utc_sec;
                p_fix_msg->header.stamp.nanosec = utc_ns;
                p_fix_msg->status.status = gps_status;
                p_fix_msg->latitude = lat;
                p_fix_msg->longitude = lon;
                p_fix_msg->altitude = alt;
                p_fix_msg->position_covariance[0] = position_std[0] * position_std[0];
                p_fix_msg->position_covariance[4] = position_std[1] * position_std[1];
                p_fix_msg->position_covariance[8] = position_std[2] * position_std[2];
                p_fix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
                pImpl->fix_pub->publish(std::move(p_fix_msg));
            }
        }
    });
}  // namespace ins570_decode

DecodeNode::~DecodeNode() {
    if (pImpl) {
        pImpl->req_stop = true;
        if (pImpl->rx_thread.joinable()) pImpl->rx_thread.join();
    }
}

}  // namespace ins570_decode

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ins570_decode::DecodeNode);