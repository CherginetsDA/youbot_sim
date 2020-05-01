#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "std_msgs/Float64MultiArray.h"

ros::Subscriber subPositionCmd;
ros::Subscriber subPosition;
ros::Subscriber subGripper;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* acGripper;

void positionCallback(const brics_actuator::JointPositions& jointPositions) {
	control_msgs::FollowJointTrajectoryGoal msg;

	trajectory_msgs::JointTrajectoryPoint point;
	for (int i = 0; i < 5; i++) { // 5 DOF
		point.positions.push_back(jointPositions.positions[i].value);
		point.velocities.push_back(0);
		point.accelerations.push_back(0);
	}
	point.time_from_start = ros::Duration(0.1);
	msg.trajectory.points.push_back(point);

	// set joint names
	for (int i = 0; i < 5; i++) {
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		msg.trajectory.joint_names.push_back(jointName.str());
	}

	// fill message header and sent it out
	msg.trajectory.header.frame_id = "arm_link_0";
	msg.trajectory.header.stamp = ros::Time::now();
	ac->sendGoal(msg);
}

void positionCallbackCmd(const std_msgs::Float64MultiArray& jointPositions) {
	control_msgs::FollowJointTrajectoryGoal msg;

	trajectory_msgs::JointTrajectoryPoint point;
	for (int i = 0; i < 5; i++) { // 5 DOF
		point.positions.push_back(jointPositions.data[i]);
		point.velocities.push_back(0);
		point.accelerations.push_back(0);
	}
	point.time_from_start = ros::Duration(0.1);
	msg.trajectory.points.push_back(point);

	// set joint names
	for (int i = 0; i < 5; i++) {
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		msg.trajectory.joint_names.push_back(jointName.str());
	}

	// fill message header and sent it out
	msg.trajectory.header.frame_id = "arm_link_0";
	msg.trajectory.header.stamp = ros::Time::now();
	ac->sendGoal(msg);
}

void gripperCallback(const brics_actuator::JointPositions& gripperPositions) {
	if (gripperPositions.positions.size() < 2)
		return;

	control_msgs::FollowJointTrajectoryGoal msg;

	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.push_back(gripperPositions.positions[0].value);
	point.positions.push_back(gripperPositions.positions[1].value);
	point.velocities.push_back(0);
	point.velocities.push_back(0);
	point.accelerations.push_back(0);
	point.accelerations.push_back(0);
	point.time_from_start = ros::Duration(0.1);
	msg.trajectory.points.push_back(point);

	msg.trajectory.joint_names.push_back("gripper_finger_joint_l");
	msg.trajectory.joint_names.push_back("gripper_finger_joint_r");

	// fill message header and sent it out
	msg.trajectory.header.frame_id = "gripper_palm_link";
	msg.trajectory.header.stamp = ros::Time::now();
	acGripper->sendGoal(msg);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "youbot_gazebo_message_wrapper");
  ros::NodeHandle nh;

	subPosition = nh.subscribe("arm_1/arm_controller/position_command", 1000, &positionCallback);
  subPositionCmd = nh.subscribe("arm_1/arm_controller/cmd_position_command", 1000, &positionCallbackCmd);
	subGripper = nh.subscribe("arm_1/gripper_controller/position_command", 1000, &gripperCallback);
	ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_1/arm_controller/follow_joint_trajectory", true);
	acGripper = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_1/gripper_controller/follow_joint_trajectory", true);

	ros::spin();

	return 0;
}
