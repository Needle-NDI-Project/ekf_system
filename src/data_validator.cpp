#include "ekf_system/data_validator.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp> // If needed for logging

namespace ekf_system
{

    DataValidator::DataValidator()
        : max_position_(10.0),        // 10 meters max position
          max_velocity_(2.0),         // 2 m/s max velocity
          max_quaternion_norm_(0.01), // 1% deviation from unit quaternion
          max_time_delay_(1.0)        // e.g., 1 second max delay
    {
    }

    bool DataValidator::validatePose(const geometry_msgs::msg::Pose &pose, std::string &error_msg)
    {
        // Check position validity
        if (!isPositionValid(pose.position.x, pose.position.y, pose.position.z, error_msg))
        {
            return false;
        }

        // Check quaternion validity
        if (!isQuaternionValid(pose.orientation.w, pose.orientation.x,
                               pose.orientation.y, pose.orientation.z, error_msg))
        {
            return false;
        }

        return true;
    }

    bool DataValidator::validateTwist(const geometry_msgs::msg::Twist &twist, std::string &error_msg)
    {
        // Check linear velocity
        if (!isVelocityValid(twist.linear.x, twist.linear.y, twist.linear.z, error_msg))
        {
            error_msg = "Linear velocity: " + error_msg;
            return false;
        }

        // Check angular velocity (here we use the same threshold, but you can separate them if needed)
        if (!isVelocityValid(twist.angular.x, twist.angular.y, twist.angular.z, error_msg))
        {
            error_msg = "Angular velocity: " + error_msg;
            return false;
        }

        return true;
    }

    bool DataValidator::validatePoseArray(const std::vector<geometry_msgs::msg::Pose> &poses,
                                          const std::vector<bool> &inbound,
                                          std::string &error_msg)
    {
        if (poses.empty())
        {
            error_msg = "Empty pose array";
            return false;
        }

        if (poses.size() != inbound.size())
        {
            error_msg = "Mismatched poses and inbound flags";
            return false;
        }

        // Validate each pose that is marked as inbound
        for (size_t i = 0; i < poses.size(); i++)
        {
            if (inbound[i])
            {
                if (!validatePose(poses[i], error_msg))
                {
                    error_msg = "Invalid pose at index " + std::to_string(i) + ": " + error_msg;
                    return false;
                }
            }
        }
        return true;
    }

    bool DataValidator::validateTimeStamp(const rclcpp::Time &msg_time,
                                          const rclcpp::Time &current_time,
                                          std::string &error_msg)
    {
        double delay = (current_time - msg_time).seconds();
        if (delay > max_time_delay_)
        {
            error_msg = "Message too old: " + std::to_string(delay) + "s delay";
            return false;
        }
        return true;
    }

    // Private helper methods
    bool DataValidator::isQuaternionValid(double w, double x, double y, double z, std::string &error_msg)
    {
        if (std::isnan(w) || std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            error_msg = "Quaternion contains NaN values";
            return false;
        }

        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (std::abs(norm - 1.0) > max_quaternion_norm_)
        {
            error_msg = "Quaternion norm deviation exceeds threshold";
            return false;
        }

        return true;
    }

    bool DataValidator::isPositionValid(double x, double y, double z, std::string &error_msg)
    {
        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            error_msg = "Position contains NaN values";
            return false;
        }

        double magnitude = std::sqrt(x * x + y * y + z * z);
        if (magnitude > max_position_)
        {
            error_msg = "Position magnitude exceeds maximum allowed value";
            return false;
        }

        return true;
    }

    bool DataValidator::isVelocityValid(double vx, double vy, double vz, std::string &error_msg)
    {
        if (std::isnan(vx) || std::isnan(vy) || std::isnan(vz))
        {
            error_msg = "Velocity contains NaN values";
            return false;
        }

        double magnitude = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (magnitude > max_velocity_)
        {
            error_msg = "Velocity magnitude exceeds maximum allowed value";
            return false;
        }

        return true;
    }

} // namespace ekf_system
