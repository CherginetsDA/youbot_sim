#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <Eigen/Dense>

ros::Subscriber subPositionCmd;
ros::Subscriber subPosition;
ros::Subscriber subGripper;
ros::Subscriber subCartesianPointTask;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* acGripper;
Eigen::VectorXf offset(5);

void cartesian_callback(const geometry_msgs::Twist cmsg)
{
  double x = cmsg.linear.x; // x of point in Cartesian coordinate system
  double y = cmsg.linear.y; // y of point in Cartesian coordinate system
  double z = cmsg.linear.z; // z of point in Cartesian coordinate system
  double alpha = cmsg.angular.x; //  angle which set tool orientation,
  double beta = cmsg.angular.y;  //  angle of last frame of arm, also set tool orientation
  // cmsg.angular.z didn't use
  double g; //variable defining the plane in which the axis of the joints will be
  if (x <= 0){
    g = sqrt(x*x + y*y) + 0.033 - cos(alpha)*0.13;
  } else {
    g = sqrt(x*x + y*y) - 0.033 - cos(alpha)*0.13;
  }
  double l1,l2,l3;
  z -= 0.115  - sin(alpha)*0.13; // разница по оси з  между вторым и последним суставом
  double tthi = atan2(z,g);
  l1 = 0.155;
  l2 = 0.135;
  l3 = sqrt (z*z + g*g);
  double thi = acos((l1*l1 + l2 * l2 - l3*l3)/(2*l1*l2));

  Eigen::VectorXf q(5);
  if (x > 0)
  {
    q[0] = atan(y/x);
    q[1] = M_PI/2 - asin(l2*sin(thi)/l3) - tthi;
    q[2] = M_PI - thi;
    q[3] = tthi - asin(l1*sin(thi)/l3) + alpha;
    q[4] = beta;
  } else {
    q[0] =  atan(y/x);
    q[1] = -(M_PI/2 - asin(l2*sin(thi)/l3) - tthi);
    q[2] = -(M_PI - thi);
    q[3] = -(tthi - asin(l1*sin(thi)/l3) + alpha);
    q[4] = beta;
  }

  if (sqrt(z*z + g*g) < 0.285) // sending message
  {

    control_msgs::FollowJointTrajectoryGoal msg;
  	trajectory_msgs::JointTrajectoryPoint point;

    q += offset;

    for (int i = 0; i < 5; i++) {
  		point.positions.push_back(q[i]);
  		point.velocities.push_back(0);
  		point.accelerations.push_back(0);
  	}
  	point.time_from_start = ros::Duration(0.1);
  	msg.trajectory.points.push_back(point);

  	for (int i = 0; i < 5; i++) {
  		std::stringstream jointName;
  		jointName << "arm_joint_" << (i + 1);
  		msg.trajectory.joint_names.push_back(jointName.str());
  	}

  	msg.trajectory.header.frame_id = "arm_link_0";
  	msg.trajectory.header.stamp = ros::Time::now();
  	ac->sendGoal(msg);
  } else
  {
    ROS_INFO("The manipulator can't get this point.");
  }
}

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
	offset << 169,65,-146,102.5, 167.5;
  offset *= M_PI/180;
  ros::init (argc, argv, "youbot_gazebo_message_wrapper");
  ros::NodeHandle nh;

	subPosition = nh.subscribe("arm_1/arm_controller/position_command", 1000, &positionCallback);
  subPositionCmd = nh.subscribe("arm_1/arm_controller/cmd_position_command", 1000, &positionCallbackCmd);
	subGripper = nh.subscribe("arm_1/gripper_controller/position_command", 1000, &gripperCallback);
	subCartesianPointTask = nh.subscribe("kinematic/add_cartesian_point_to_task",1000,&cartesian_callback);
	ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_1/arm_controller/follow_joint_trajectory", true);
	acGripper = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_1/gripper_controller/follow_joint_trajectory", true);

	ros::spin();

	return 0;
}
