#ifndef EKF_SYSTEM_DATA_VALIDATOR_HPP_
#define EKF_SYSTEM_DATA_VALIDATOR_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp> // <--- IMPORTANT: for rclcpp::Time
#include <string>
#include <vector>

namespace ekf_system
{
    class DataValidator
    {
    public:
        DataValidator();
        ~DataValidator() = default;

        // Validation methods for different data types
        bool validatePose(const geometry_msgs::msg::Pose &pose, std::string &error_msg);
        bool validateTwist(const geometry_msgs::msg::Twist &twist, std::string &error_msg);
        bool validatePoseArray(const std::vector<geometry_msgs::msg::Pose> &poses,
                               const std::vector<bool> &inbound,
                               std::string &error_msg);

        // Timestamp validation (optional if we do not need time checks)
        bool validateTimeStamp(const rclcpp::Time &msg_time,
                               const rclcpp::Time &current_time,
                               std::string &error_msg);

        // Settings for validation thresholds
        void setMaxPosition(double max_pos) { max_position_ = max_pos; }
        void setMaxVelocity(double max_vel) { max_velocity_ = max_vel; }
        void setMaxQuaternionNorm(double max_quaternion_norm) { max_quaternion_norm_ = max_quaternion_norm; }
        void setMaxTimeDelay(double max_delay) { max_time_delay_ = max_delay; }

    private:
        // Validation thresholds
        double max_position_;        // Maximum allowed position magnitude
        double max_velocity_;        // Maximum allowed velocity magnitude
        double max_quaternion_norm_; // Maximum allowed quaternion norm deviation from 1.0
        double max_time_delay_;      // Maximum allowed delay in seconds

        // Helper methods
        bool isQuaternionValid(double w, double x, double y, double z, std::string &error_msg);
        bool isPositionValid(double x, double y, double z, std::string &error_msg);
        bool isVelocityValid(double vx, double vy, double vz, std::string &error_msg);
    };

} // namespace ekf_system

#endif // EKF_SYSTEM_DATA_VALIDATOR_HPP_
