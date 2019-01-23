#ifndef ORIGINALMANIPULATION_H_
#define ORIGINALMANIPULATION_H_

#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <manipulator_h_base_module_msgs/KinematicsPose.h>
#include <manipulator_h_base_module_msgs/SetKinematicMoveSpeed.h>

using namespace std;

typedef struct KinematicPose{
  double x_;
  double y_;
  double z_;
  double roll_;
  double pitch_;
  double yaw_;
} kpose;

class OriginalManipulation{
public:
  OriginalManipulation();
  ~OriginalManipulation();
  void Init();
  void Proc();
  void SetMode();
  void SetInitPose();
  void SetKinematicsPose(kpose pose);
  void SetKinematicsPose(double x_, double y_, double z_, double roll_, double pitch_, double yaw_);
  Eigen::MatrixXd rotationX( double angle );
  Eigen::MatrixXd rotationY( double angle );
  Eigen::MatrixXd rotationZ( double angle );

  Eigen::MatrixXd rotation2rpy( Eigen::MatrixXd rotation );
  Eigen::MatrixXd rpy2rotation( double roll, double pitch, double yaw );

  Eigen::Quaterniond rpy2quaternion( double roll, double pitch, double yaw );
  Eigen::Quaterniond rotation2quaternion( Eigen::MatrixXd rotation );

  Eigen::MatrixXd quaternion2rpy( Eigen::Quaterniond quaternion );
  Eigen::MatrixXd quaternion2rotation( Eigen::Quaterniond quaternion );    

  void sendIniPoseMsg(std_msgs::String msg);
  void sendSetModeMsg(std_msgs::String msg);

  void sendKinematicsPoseMsg(manipulator_h_base_module_msgs::KinematicsPose msg);
  void sendKinematicMoveSpeed(double tol_, double mov_time_);

private:
  ros::Publisher kinematics_pose_msg_pub_;
  ros::Publisher ini_pose_msg_pub_;
  ros::Publisher set_mode_msg_pub_;
  ros::Subscriber stetus_msg_sub_;
  ros::ServiceClient set_kinematic_move_speed_client_;
};

#endif
