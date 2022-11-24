#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros_com/frame_transforms.h>

using namespace px4_ros_com::frame_transforms;
using namespace px4_msgs::msg;


class OdometryListener : public rclcpp::Node
{
public:
	OdometryListener() : Node("Odometry_listener")
	{
		publisher_ = this->create_publisher<VehicleVisualOdometry>("fmu/vehicle_visual_odometry/in", 10);
		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"visual_slam/tracking/odometry", 10,
			[this](const nav_msgs::msg::Odometry::UniquePtr msg)
			{
				auto odometry_msg = VehicleVisualOdometry();
				odometry_msg.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

				//Position
				// Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
				// pos = transform_static_frame(pos, StaticTF::ENU_TO_NED);

				// odometry_msg.x = pos(1);
				// odometry_msg.y = pos(2);
				// odometry_msg.z = pos(3);
				
				float prev_y = msg->pose.pose.position.y;
				float prev_z = msg->pose.pose.position.z;

				odometry_msg.x = msg->pose.pose.position.x;
				odometry_msg.y = -prev_y;
				odometry_msg.z = -prev_z;

				//Quaternion
				auto quaternion = Eigen::Quaterniond(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
				quaternion = transform_orientation(quaternion, StaticTF::ENU_TO_NED);

				odometry_msg.q[0] = quaternion.w();
				odometry_msg.q[1] = quaternion.x();
				odometry_msg.q[2] = quaternion.y();
				odometry_msg.q[3] = quaternion.z();

				odometry_msg.q_offset[0] = 1;

				odometry_msg.vx = msg->twist.twist.linear.x;
				odometry_msg.vy = msg->twist.twist.linear.y;
				odometry_msg.vz = msg->twist.twist.linear.z;

				odometry_msg.rollspeed = msg->twist.twist.angular.x;
				odometry_msg.pitchspeed = msg->twist.twist.angular.y;
				odometry_msg.yawspeed = msg->twist.twist.angular.z;

				odometry_msg.local_frame = 0;
				odometry_msg.velocity_frame = 1;

				this->publisher_->publish(odometry_msg);

				//Covariance
				// Covariance6d covariancePose = transform_frame(msg->pose.covariance, quaternion);
				// Covariance6d covarianceVelocity = transform_frame(msg->twist.covariance, quaternion);
				
				// utils::types::covariance_urt_to_array<Covariance6d, 21>(covariancePose, odometry_msg.pose_covariance);
				// utils::types::covariance_urt_to_array<Covariance6d, 21>(covarianceVelocity, odometry_msg.pose_covariance);
				
				// //Rotation
				// auto msgLinear = msg->twist.twist.linear;
				// Eigen::Vector3d linear = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

				// auto transTwist = enu_to_ned_local_frame<geometry_msgs::msg::Vector3>(msg->twist.twist.linear);
				// auto transAngular = enu_to_ned_local_frame<geometry_msgs::msg::Vector3>(msg->twist.twist.angular);

				// odometry_msg.vx = transTwist.x;
				// odometry_msg.vy = transTwist.y;
				// odometry_msg.vz = transTwist.z;

				// odometry_msg.rollspeed = transAngular.x;
				// odometry_msg.pitchspeed = transAngular.y;
				// odometry_msg.yawspeed = transAngular.z;

				// odometry_msg.local_frame = 0;
				// odometry_msg.velocity_frame = 1;

				// this->publisher_->publish(odometry_msg);
			});
	}

private:
	rclcpp::Publisher<VehicleVisualOdometry>::SharedPtr publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting odometry message translation node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdometryListener>());

	rclcpp::shutdown();
	return 0;
}
