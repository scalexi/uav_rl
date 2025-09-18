// odom_converter.cpp

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <array>

class OdomConverter : public rclcpp::Node
{
public:
    OdomConverter() : Node("odom_converter")
    {
        // Ensure sim time is used
        this->declare_parameter("use_sim_time", true);
        
        // Subscribe to PX4 odometry (NED frame)
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&OdomConverter::listener_callback, this, std::placeholders::_1)
        );

        // Publish ROS odometry (ENU frame)
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Frame names
        odom_frame_ = "odom";
        base_frame_ = "base_link";

        // =====================================================
        // COORDINATE TRANSFORMATION SETTINGS
        // =====================================================
        // PX4 uses NED frame: X=North, Y=East, Z=Down; Body: FRD (Forward-Right-Down)
        // ROS uses ENU frame: X=East, Y=North, Z=Up; Body: FLU (Forward-Left-Up)
        
        // World frame transformation (NED to ENU)
        world_transform_ << 0, 1, 0,   // X_enu = Y_ned (East)
                            1, 0, 0,   // Y_enu = X_ned (North)
                            0, 0, -1;  // Z_enu = -Z_ned (Up)

        // Body frame transformation (FRD to FLU)
        body_transform_ << 1, 0, 0,    // X_flu = X_frd (Forward)
                           0, -1, 0,   // Y_flu = -Y_frd (Left = -Right)
                           0, 0, -1;   // Z_flu = -Z_frd (Up = -Down)
    }

private:
    void listener_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        // --- Position transformation (NED to ENU) ---
        Eigen::Vector3d position_ned(msg->position[0], msg->position[1], msg->position[2]);
        Eigen::Vector3d position_enu = transform_vector(position_ned, world_transform_);
        odom.pose.pose.position.x = position_enu[0];
        odom.pose.pose.position.y = position_enu[1];
        odom.pose.pose.position.z = position_enu[2];

        // --- Orientation transformation (NED/FRD to ENU/FLU) ---
        std::array<double, 4> q_px4 = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
        auto q_enu = transform_orientation(q_px4);
        odom.pose.pose.orientation.x = q_enu[0];
        odom.pose.pose.orientation.y = q_enu[1];
        odom.pose.pose.orientation.z = q_enu[2];
        odom.pose.pose.orientation.w = q_enu[3];

        // --- Velocity transformation (based on velocity_frame) ---
        Eigen::Vector3d velocity_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        Eigen::Vector3d velocity_enu;
        if (msg->velocity_frame == 1) {  // VELOCITY_FRAME_NED
            velocity_enu = transform_vector(velocity_ned, world_transform_);
        } else if (msg->velocity_frame == 3) {  // VELOCITY_FRAME_BODY_FRD
            velocity_enu = transform_vector(velocity_ned, body_transform_);
        } else {
            velocity_enu = velocity_ned;  // Fallback, though unlikely
        }
        odom.twist.twist.linear.x = velocity_enu[0];
        odom.twist.twist.linear.y = velocity_enu[1];
        odom.twist.twist.linear.z = velocity_enu[2];

        // --- Angular velocity transformation (FRD to FLU) ---
        Eigen::Vector3d av_frd(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);
        Eigen::Vector3d av_flu = transform_vector(av_frd, body_transform_);
        odom.twist.twist.angular.x = av_flu[0];
        odom.twist.twist.angular.y = av_flu[1];
        odom.twist.twist.angular.z = av_flu[2];

        // --- Covariances ---
        std::array<double, 6> pose_diagonal = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
        std::array<double, 6> twist_diagonal = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
        
        odom.pose.covariance = create_covariance_matrix(pose_diagonal);
        odom.twist.covariance = create_covariance_matrix(twist_diagonal);

        // Publish /odom
        publisher_->publish(odom);

        // Broadcast TF (odom → base_link)
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = odom.header.stamp;
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_frame_;
        t.transform.translation.x = odom.pose.pose.position.x;
        t.transform.translation.y = odom.pose.pose.position.y;
        t.transform.translation.z = odom.pose.pose.position.z;
        t.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }

    std::array<double, 36> create_covariance_matrix(const std::array<double, 6>& diagonal_values)
    {
        std::array<double, 36> cov{};
        cov.fill(0.0);
        for (int i = 0; i < 6; ++i) {
            cov[i * 6 + i] = diagonal_values[i];
        }
        return cov;
    }

    Eigen::Vector3d transform_vector(const Eigen::Vector3d& px4_vector, const Eigen::Matrix3d& transform_matrix)
    {
        return transform_matrix * px4_vector;
    }

    std::array<double, 4> transform_orientation(const std::array<double, 4>& px4_quaternion)
    {
        // Convert PX4 quaternion (w, x, y, z) to tf2 quaternion
        tf2::Quaternion q_ned_frd(px4_quaternion[1], px4_quaternion[2], px4_quaternion[3], px4_quaternion[0]);
        
        // Get rotation matrix from quaternion
        tf2::Matrix3x3 R_ned_frd(q_ned_frd);
        
        // Convert tf2::Matrix3x3 to Eigen::Matrix3d
        Eigen::Matrix3d R_ned_frd_eigen;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_ned_frd_eigen(i, j) = R_ned_frd[i][j];
            }
        }
        
        // Apply transformation: R_enu_flu = world_transform @ R_ned_frd @ body_transform
        Eigen::Matrix3d R_enu_flu = world_transform_ * R_ned_frd_eigen * body_transform_;
        
        // Convert back to tf2::Matrix3x3
        tf2::Matrix3x3 R_enu_flu_tf2;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_enu_flu_tf2[i][j] = R_enu_flu(i, j);
            }
        }
        
        // Convert to quaternion
        tf2::Quaternion q_enu_flu;
        R_enu_flu_tf2.getRotation(q_enu_flu);
        
        // Ensure quaternion has positive w (convention)
        if (q_enu_flu.w() < 0) {
            q_enu_flu = -q_enu_flu;
        }
        
        return {q_enu_flu.x(), q_enu_flu.y(), q_enu_flu.z(), q_enu_flu.w()};
    }

    // Subscribers and publishers
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Frame names
    std::string odom_frame_;
    std::string base_frame_;

    // Transformation matrices
    Eigen::Matrix3d world_transform_;
    Eigen::Matrix3d body_transform_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomConverter>());
    rclcpp::shutdown();
    return 0;
}
