#include "ekf_system/ekf_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm> // for std::count

using namespace std::chrono_literals;

namespace ekf_system
{

    EKFNode::EKFNode(const rclcpp::NodeOptions &options)
        : Node("ekf_node", options)
    {
        // Create callback groups
        callback_group_subscribers_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_services_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Initialize node components
        loadParameters();
        initializeEKF();
        initializePublishers();
        initializeSubscribers();
        initializeServices();

        // Create publishing timer
        auto publish_interval = std::chrono::duration<double>(1.0 / publish_rate_);
        publish_timer_ = create_wall_timer(
            publish_interval,
            std::bind(&EKFNode::publishTimerCallback, this));

        RCLCPP_INFO(get_logger(), "EKF node initialized successfully");
    }

    void EKFNode::loadParameters()
    {
        // Declare and load parameters with default values
        this->declare_parameter("publish_rate", 50.0);
        this->declare_parameter("world_frame_id", "world");
        this->declare_parameter("use_sim_time", false);

        publish_rate_ = this->get_parameter("publish_rate").as_double();
        world_frame_id_ = this->get_parameter("world_frame_id").as_string();
        use_sim_time_ = this->get_parameter("use_sim_time").as_bool();

        // Additional EKF parameters can be loaded here if needed
    }

    void EKFNode::initializeEKF()
    {
        ekf_ = std::make_shared<ExtendedKalmanFilter>();
        validator_ = std::make_shared<DataValidator>();

        // Example: load initial state from parameter or config
        std::vector<double> initial_state = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // 13 states: position(3), quaternion(4), linear vel(3), angular vel(3)

        Eigen::VectorXd x0 = Eigen::Map<Eigen::VectorXd>(initial_state.data(), initial_state.size());
        ekf_->setInitialState(x0);
    }

    void EKFNode::initializePublishers()
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ekf_state_estimate", 10);
    }

    void EKFNode::initializeSubscribers()
    {
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_subscribers_;

        // Subscribe to RigidArray for NDI data
        rigid_sub_ = this->create_subscription<ndi_msgs::msg::RigidArray>(
            "/ndi_tracker/state", 10,
            std::bind(&EKFNode::rigidCallback, this, std::placeholders::_1),
            options);

        // Subscribe to cmd_vel for velocity commands
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10,
            std::bind(&EKFNode::cmdVelCallback, this, std::placeholders::_1),
            options);
    }

    void EKFNode::initializeServices()
    {
        get_state_srv_ = this->create_service<srv::GetEKFState>(
            "/ekf_system/get_filter_state",
            std::bind(&EKFNode::handleGetState, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_services_);

        reset_srv_ = this->create_service<srv::ResetFilter>(
            "/ekf_system/reset_filter",
            std::bind(&EKFNode::handleReset, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_services_);
    }

    void EKFNode::rigidCallback(const ndi_msgs::msg::RigidArray::SharedPtr msg)
    {
        // Validate time stamp
        std::string error_msg;
        if (!validator_->validateTimeStamp(msg->header.stamp, this->now(), error_msg))
        {
            RCLCPP_WARN(get_logger(), "Time validation failed: %s", error_msg.c_str());
            return;
        }

        // Validate poses
        if (!validator_->validatePoseArray(msg->poses, msg->inbound, error_msg))
        {
            RCLCPP_WARN(get_logger(), "Pose validation failed: %s", error_msg.c_str());
            return;
        }

        // Construct measurement from RigidArray
        auto measurement_data = constructMeasurement(msg);

        // Update filter
        ekf_->update(measurement_data.measurement, measurement_data.H, measurement_data.R);
    }

    EKFNode::MeasurementData EKFNode::constructMeasurement(const ndi_msgs::msg::RigidArray::SharedPtr &msg)
    {
        MeasurementData data;
        std::vector<bool> valid_measurements;
        std::vector<geometry_msgs::msg::Pose> transformed_poses;

        // Transform each valid pose to world frame
        for (size_t i = 0; i < msg->poses.size(); i++)
        {
            if (msg->inbound[i])
            {
                geometry_msgs::msg::Pose transformed_pose;
                if (transformPose(msg->poses[i], transformed_pose))
                {
                    transformed_poses.push_back(transformed_pose);
                    valid_measurements.push_back(true);
                }
                else
                {
                    valid_measurements.push_back(false);
                }
            }
            else
            {
                valid_measurements.push_back(false);
            }
        }

        // Count how many valid transformations
        size_t valid_count = std::count(valid_measurements.begin(), valid_measurements.end(), true);
        // Each pose = position(3) + orientation(4) = 7
        size_t meas_size = valid_count * 7;

        data.measurement = Eigen::VectorXd::Zero(meas_size);
        data.H = Eigen::MatrixXd::Zero(meas_size, ekf_->getState().size());
        data.R = Eigen::MatrixXd::Identity(meas_size, meas_size);

        size_t current_index = 0;
        for (size_t i = 0; i < transformed_poses.size(); i++)
        {
            if (valid_measurements[i])
            {
                const auto &pose = transformed_poses[i];
                // Fill measurement vector
                data.measurement.segment<3>(current_index) << pose.position.x, pose.position.y, pose.position.z;
                data.measurement.segment<4>(current_index + 3) << pose.orientation.w, pose.orientation.x,
                    pose.orientation.y, pose.orientation.z;

                // Fill H matrix to map measurement to state (pos @ indices 0..2, quat @ 3..6)
                data.H.block<3, 3>(current_index, 0) = Eigen::Matrix3d::Identity();
                data.H.block<4, 4>(current_index + 3, 3) = Eigen::Matrix4d::Identity();

                // Example measurement noise
                double position_variance = 0.001;
                double orientation_variance = 0.001;
                data.R.block<3, 3>(current_index, current_index) *= position_variance;
                data.R.block<4, 4>(current_index + 3, current_index + 3) *= orientation_variance;

                current_index += 7;
            }
        }

        return data;
    }

    bool EKFNode::transformPose(const geometry_msgs::msg::Pose &pose_in, geometry_msgs::msg::Pose &pose_out)
    {
        // TODO: If you have TF transforms, apply them here.
        // For now, just pass it through as if pose_in is already in 'world' frame.
        pose_out = pose_in;
        return true;
    }

    void EKFNode::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        std::string error_msg;
        if (!validator_->validateTwist(msg->twist, error_msg))
        {
            RCLCPP_WARN(get_logger(), "Invalid control input: %s", error_msg.c_str());
            return;
        }

        // Convert twist to control input
        // example: [vx, vy, vz, wx, wy, wz]
        Eigen::VectorXd control(6);
        control << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
            msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;

        // Calculate dt
        auto current_time = this->now();
        double dt = (last_update_time_.nanoseconds() > 0)
                        ? (current_time - last_update_time_).seconds()
                        : 0.0;
        last_update_time_ = current_time;

        if (dt > 0.0)
        {
            ekf_->predict(control, dt);
        }
    }

    void EKFNode::publishTimerCallback()
    {
        auto state = ekf_->getState();
        auto covariance = ekf_->getStateCovariance();

        auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
        msg->header.stamp = this->now();
        msg->header.frame_id = world_frame_id_;

        // Fill pose (assuming: 0..2 pos, 3..6 quaternion)
        msg->pose.pose.position.x = state(0);
        msg->pose.pose.position.y = state(1);
        msg->pose.pose.position.z = state(2);
        msg->pose.pose.orientation.w = state(3);
        msg->pose.pose.orientation.x = state(4);
        msg->pose.pose.orientation.y = state(5);
        msg->pose.pose.orientation.z = state(6);

        // If you only have a 13-state vector, fill 6x6 portion of covariance
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                msg->pose.covariance[i * 6 + j] = covariance(i, j);
            }
        }

        pose_pub_->publish(std::move(msg));
    }

    void EKFNode::handleGetState(
        const std::shared_ptr<srv::GetEKFState::Request> /*request*/,
        std::shared_ptr<srv::GetEKFState::Response> response)
    {
        auto state = ekf_->getState();
        auto covariance = ekf_->getStateCovariance();

        // Fill pose
        response->pose.pose.position.x = state(0);
        response->pose.pose.position.y = state(1);
        response->pose.pose.position.z = state(2);
        response->pose.pose.orientation.w = state(3);
        response->pose.pose.orientation.x = state(4);
        response->pose.pose.orientation.y = state(5);
        response->pose.pose.orientation.z = state(6);

        // Fill twist if state > 7
        if (state.size() > 7)
        {
            response->twist.twist.linear.x = state(7);
            response->twist.twist.linear.y = state(8);
            response->twist.twist.linear.z = state(9);
            response->twist.twist.angular.x = state(10);
            response->twist.twist.angular.y = state(11);
            response->twist.twist.angular.z = state(12);
        }

        // Flatten full covariance
        response->state_covariance.resize(covariance.size());
        Eigen::Map<Eigen::MatrixXd>(response->state_covariance.data(),
                                    covariance.rows(), covariance.cols()) = covariance;
    }

    void EKFNode::handleReset(
        const std::shared_ptr<srv::ResetFilter::Request> request,
        std::shared_ptr<srv::ResetFilter::Response> response)
    {
        if (request->reset_to_default)
        {
            ekf_->reset();
            response->success = true;
            response->message = "Filter reset to default state";
        }
        else
        {
            // Convert provided pose to state vector (size 13)
            Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(13);

            // Position
            initial_state(0) = request->initial_pose.position.x;
            initial_state(1) = request->initial_pose.position.y;
            initial_state(2) = request->initial_pose.position.z;

            // Orientation
            initial_state(3) = request->initial_pose.orientation.w;
            initial_state(4) = request->initial_pose.orientation.x;
            initial_state(5) = request->initial_pose.orientation.y;
            initial_state(6) = request->initial_pose.orientation.z;

            // Twist
            initial_state(7) = request->initial_twist.linear.x;
            initial_state(8) = request->initial_twist.linear.y;
            initial_state(9) = request->initial_twist.linear.z;
            initial_state(10) = request->initial_twist.angular.x;
            initial_state(11) = request->initial_twist.angular.y;
            initial_state(12) = request->initial_twist.angular.z;

            ekf_->reset(initial_state);
            response->success = true;
            response->message = "Filter reset to provided state";
        }
    }

} // namespace ekf_system

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ekf_system::EKFNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}