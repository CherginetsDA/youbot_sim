#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "std_msgs/Empty.h"

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac;

//TODO: add home position to the rosparam

void HomeCallback(const std_msgs::Empty& empty_msg)
{
  control_msgs::FollowJointTrajectoryGoal msg;

  trajectory_msgs::JointTrajectoryPoint point;
  for (int i = 0; i < 5; i++) { // 5 DOF
          point.positions.push_back(0);
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
  ROS_INFO("YouBot come home.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_home");
  ros::NodeHandle nh;
  ros::Subscriber subPosition = nh.subscribe("home", 1, &HomeCallback);
  ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_1/arm_controller/follow_joint_trajectory", true);
  ros::spin();

  return 0;
}
