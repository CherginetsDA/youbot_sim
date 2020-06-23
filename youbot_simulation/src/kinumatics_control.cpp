#include <iostream>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/mdl/NloptInverseKinematics.h>
#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "geometry_msgs/Twist.h"
#include <chrono>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

Eigen::VectorXf offset(5);
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac;

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

int main(int argc, char** argv)
{
  // Prepaer!!!
  offset << 169,65,-146,102.5, 167.5;
  offset *= M_PI/180;

  ros::init(argc,argv,"kinematics");
  ros::NodeHandle nh;
  ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_1/arm_controller/follow_joint_trajectory", true);
  ros::Subscriber subCartesianPointTask = nh.subscribe("kinematic/go_to_solve_ik",1000,&cartesian_callback);
  ros::spin();

  return 0;
}
