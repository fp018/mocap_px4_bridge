#include <chrono>
#include <memory>
#include "Eigen/Dense"
#include "utils.h"
#include "rclcpp/rclcpp.hpp"

#include <px4_msgs/msg/vehicle_odometry.hpp>

#ifdef PX4_ROS_TIMESYNC
	#include <px4_msgs/msg/timesync.hpp>
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;


class MocapPX4Bridge : public rclcpp::Node
{
public:
	MocapPX4Bridge() : Node("mocap_px4_bridge") {
		this->declare_parameter("mocap_topic", "/Robot_1/pose");
		this->declare_parameter("px4_topic", "/fmu/in/vehicle_visual_odometry");

		const std::string mocap_topic = this->get_parameter("mocap_topic").as_string();
		const std::string px4_topic = this->get_parameter("px4_topic").as_string();

		RCLCPP_INFO(get_logger(), "mocap_topic: %s", mocap_topic.c_str());
		RCLCPP_INFO(get_logger(), "px4_topic: %s", px4_topic.c_str());

		poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(mocap_topic, 10, std::bind(&MocapPX4Bridge::poseCallback, this, _1));
		odomPub = this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic, 10);
	}

private:
	void poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odomPub;
};

void MocapPX4Bridge::poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	RCLCPP_INFO_ONCE(get_logger(), "Recived first msg from optitrack.");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", poseMsg->pose.position.x, 
					poseMsg->pose.position.y, poseMsg->pose.position.z);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", poseMsg->pose.orientation.w,
					poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
	

	// Optitrack (Y - Up ) to frame to NED conversion
	// Ref: https://docs.px4.io/main/en/ros/external_position_estimation.html

	Eigen::Vector3d p_opt; // = Zeros;
	Eigen::Vector4d q_opt; // = Zeros;
	p_opt << poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z;
	q_opt << poseMsg->pose.orientation.w, poseMsg->pose.orientation.x,  poseMsg->pose.orientation.y,  poseMsg->pose.orientation.z;
	
	
	Eigen::Matrix3d R_o_ned;
	R_o_ned <<  1,  0, 0,
			    0,  0, 1,
			    0, -1, 0;
	
	// Position 
	Eigen::Vector3d p_opt2_ned = R_o_ned*p_opt;


	px4_msgs::msg::VehicleOdometry odomMsg;

	odomMsg.pose_frame = odomMsg.POSE_FRAME_FRD;
	odomMsg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	odomMsg.timestamp_sample = odomMsg.timestamp;

	odomMsg.position[0] = p_opt2_ned[0];
	odomMsg.position[1] = p_opt2_ned[1];
	odomMsg.position[2] = p_opt2_ned[2];


	odomMsg.q[0] = q_opt[0];
	odomMsg.q[1] = q_opt[1];
	odomMsg.q[2] = q_opt[3];
	odomMsg.q[3] = -q_opt[2];
	Eigen::Vector4d final_q(odomMsg.q[0],odomMsg.q[1], odomMsg.q[2], odomMsg.q[3]);

	Eigen::Matrix3d final_R = utilities::QuatToMat(final_q);
	Eigen::Vector3d euler = utilities::R2XYZ(final_R);


	odomPub -> publish(odomMsg);
	RCLCPP_INFO_ONCE(get_logger(), "Sent to PX4 as:");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", odomMsg.position[0], odomMsg.position[1], odomMsg.position[2]);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", odomMsg.q[0], odomMsg.q[1], odomMsg.q[2], odomMsg.q[3]);
	RCLCPP_INFO_ONCE(get_logger(), "rpy: %f, %f, %f", euler[0],euler[1], euler[2]);
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapPX4Bridge>());
	rclcpp::shutdown();
	return 0;
}
