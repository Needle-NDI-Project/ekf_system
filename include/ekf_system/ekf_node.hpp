#ifndef EKF_SYSTEM_EKF_NODE_HPP_
#define EKF_SYSTEM_EKF_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ndi_msgs/msg/rigid_array.hpp" // Changed from TrackerArray
#include "std_srvs/srv/empty.hpp"
#include "ekf_system/srv/get_ekf_state.hpp"
#include "ekf_system/srv/reset_filter.hpp"

#include "ekf_system/extended_kalman_filter.hpp"
#include "ekf_system/data_validator.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace ekf_system
{

    class EKFNode : public rclcpp::Node
    {
    public:
        explicit EKFNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        virtual ~EKFNode() = default;

    private:
        // Node components
        std::shared_ptr<ExtendedKalmanFilter> ekf_;
        std::shared_ptr<DataValidator> validator_;

        // TF Handling
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // For measurement handling
        struct MeasurementData
        {
            Eigen::VectorXd measurement;
            Eigen::MatrixXd H;
            Eigen::MatrixXd R;
        };

        // *** Updated to match RigidArray ***
        MeasurementData constructMeasurement(const ndi_msgs::msg::RigidArray::SharedPtr &msg);

        // *** Implementation of transformPose ***
        bool transformPose(const geometry_msgs::msg::Pose &pose_in, geometry_msgs::msg::Pose &pose_out);

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

        // *** Subscriptions updated to RigidArray for NDI ***
        rclcpp::Subscription<ndi_msgs::msg::RigidArray>::SharedPtr rigid_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

        // Services
        rclcpp::Service<srv::GetEKFState>::SharedPtr get_state_srv_;
        rclcpp::Service<srv::ResetFilter>::SharedPtr reset_srv_;

        // Callback groups for thread safety
        rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
        rclcpp::CallbackGroup::SharedPtr callback_group_services_;

        // Parameters
        double publish_rate_;
        std::string world_frame_id_;
        bool use_sim_time_;

        // Timers
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::Time last_update_time_;

        // Callback methods
        void rigidCallback(const ndi_msgs::msg::RigidArray::SharedPtr msg);
        void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void publishTimerCallback();

        // Service callbacks
        void handleGetState(
            const std::shared_ptr<srv::GetEKFState::Request> request,
            std::shared_ptr<srv::GetEKFState::Response> response);
        void handleReset(
            const std::shared_ptr<srv::ResetFilter::Request> request,
            std::shared_ptr<srv::ResetFilter::Response> response);

        // Helper init methods
        void loadParameters();
        void initializePublishers();
        void initializeSubscribers();
        void initializeServices();
        void initializeEKF();
    };

} // namespace ekf_system

#endif // EKF_SYSTEM_EKF_NODE_HPP_
