#include "OriginalManipulation.hpp"

OriginalManipulation::OriginalManipulation(){
  Init();
}

OriginalManipulation::~OriginalManipulation(){
  cout << "OriginalManipulation was deleted" << endl;
}

void OriginalManipulation::Init(){
  ros::NodeHandle n;

  set_mode_msg_pub_ = n.advertise<std_msgs::String>("/robotis/base/set_mode_msg", 0);
  ini_pose_msg_pub_ = n.advertise<std_msgs::String>("/robotis/base/ini_pose_msg", 0);

  kinematics_pose_msg_pub_ = n.advertise<manipulator_h_base_module_msgs::KinematicsPose>("/robotis/base/kinematics_pose_msg", 0);
  set_kinematic_move_speed_client_ = n.serviceClient<manipulator_h_base_module_msgs::SetKinematicMoveSpeed>("/robotis/base/set_kinematic_move_speed", 0);

  //Proc();
}

// void OriginalManipulation::Proc(){
//   SetMode();
//   sleep(5);

//   // SetInitPose();
//   // sleep(5);

//   SetKinematicsPose(0.1, 0.0, 0.03, 0.0, 90.0, 0.0);
//   sleep(5);
//   for(int i = 0; i < 5; i++){
//     sendKinematicMoveSpeed((double)(i+1)*0.1, 0.3/((double)(i+1)*0.1));
//     SetKinematicsPose(0.4, 0.0, 0.03, 0.0, 90.0, 0.0);
//     sleep(5);

//     SetKinematicsPose(0.1, 0.0, 0.03, 0.0, 90.0, 0.0);
//     sleep(5);
//   }

//   sendKinematicMoveSpeed(0.1, 2.0);
//   SetKinematicsPose(0.43, 0.0, 0.018, 0.0, 0.0, 0.0);
// }

void OriginalManipulation::SetMode(){
  std_msgs::String msg;
  msg.data ="set_mode";

  sendSetModeMsg( msg );
}

void OriginalManipulation::SetInitPose(){
  std_msgs::String msg;
  msg.data ="ini_pose";

  sendIniPoseMsg( msg );
}

void OriginalManipulation::sendKinematicMoveSpeed(double tol_, double mov_time_){
  ROS_INFO("Send Kinematic Move Speed Params");
  
  manipulator_h_base_module_msgs::SetKinematicMoveSpeed _set_kinematic_move_speed;

  _set_kinematic_move_speed.request.tol = tol_;
  _set_kinematic_move_speed.request.mov_time = mov_time_;

  if( set_kinematic_move_speed_client_.call( _set_kinematic_move_speed ) )
    cout << "Move speed was set" << endl;
  else
    ROS_ERROR("Failed to call service set_kinematic_move_speed");
}

void OriginalManipulation::SetKinematicsPose(kpose pose){
  manipulator_h_base_module_msgs::KinematicsPose msg;

  msg.name = "arm";

  msg.pose.position.x = pose.x_;
  msg.pose.position.y = pose.y_;
  msg.pose.position.z = pose.z_;

  double roll = pose.roll_ * M_PI / 180.0;
  double pitch = pose.pitch_ * M_PI / 180.0;
  double yaw = pose.yaw_ * M_PI / 180.0;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  sendKinematicsPoseMsg( msg );
}

void OriginalManipulation::SetKinematicsPose(double x_, double y_, double z_, double roll_, double pitch_, double yaw_){
  manipulator_h_base_module_msgs::KinematicsPose msg;

  msg.name = "arm";

  msg.pose.position.x = x_;
  msg.pose.position.y = y_;
  msg.pose.position.z = z_;

  double roll = roll_ * M_PI / 180.0;
  double pitch = pitch_ * M_PI / 180.0;
  double yaw = yaw_ * M_PI / 180.0;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  sendKinematicsPoseMsg( msg );
}

Eigen::MatrixXd OriginalManipulation::rotationX( double angle ){
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << 1.0,          0.0,           0.0,
      0.0, cos( angle ), -sin( angle ),
      0.0, sin( angle ),  cos( angle );

  return _rotation;
}

Eigen::MatrixXd OriginalManipulation::rotationY( double angle ){
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << cos( angle ), 0.0, sin( angle ),
      0.0, 1.0, 	     0.0,
      -sin( angle ), 0.0, cos( angle );

  return _rotation;
}

Eigen::MatrixXd OriginalManipulation::rotationZ( double angle ){
  Eigen::MatrixXd _rotation(3,3);

  _rotation << cos( angle ), -sin( angle ), 0.0,
      sin( angle ),  cos( angle ), 0.0,
      0.0,           0.0, 1.0;

  return _rotation;
}

Eigen::MatrixXd OriginalManipulation::rotation2rpy( Eigen::MatrixXd rotation ){
  Eigen::MatrixXd _rpy = Eigen::MatrixXd::Zero( 3 , 1 );

  _rpy.coeffRef( 0 , 0 ) = atan2( rotation.coeff( 2 , 1 ), rotation.coeff( 2 , 2 ) );
  _rpy.coeffRef( 1 , 0 ) = atan2( -rotation.coeff( 2 , 0 ), sqrt( pow( rotation.coeff( 2 , 1 ) , 2 ) + pow( rotation.coeff( 2 , 2 ) , 2 ) ) );
  _rpy.coeffRef( 2 , 0 ) = atan2 ( rotation.coeff( 1 , 0 ) , rotation.coeff( 0 , 0 ) );

  return _rpy;
}

Eigen::MatrixXd OriginalManipulation::rpy2rotation( double roll, double pitch, double yaw ){
  Eigen::MatrixXd _rotation = rotationZ( yaw ) * rotationY( pitch ) * rotationX( roll );

  return _rotation;
}

Eigen::Quaterniond OriginalManipulation::rpy2quaternion( double roll, double pitch, double yaw ){
  Eigen::MatrixXd _rotation = rpy2rotation( roll, pitch, yaw );

  Eigen::Matrix3d _rotation3d;
  _rotation3d = _rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;

  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::Quaterniond OriginalManipulation::rotation2quaternion( Eigen::MatrixXd rotation ){
  Eigen::Matrix3d _rotation3d;

  _rotation3d = rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;
  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::MatrixXd OriginalManipulation::quaternion2rpy( Eigen::Quaterniond quaternion ){
  Eigen::MatrixXd _rpy = rotation2rpy( quaternion.toRotationMatrix() );

  return _rpy;
}

Eigen::MatrixXd OriginalManipulation::quaternion2rotation( Eigen::Quaterniond quaternion ){
  Eigen::MatrixXd _rotation = quaternion.toRotationMatrix();

  return _rotation;
}

void OriginalManipulation::sendKinematicsPoseMsg( manipulator_h_base_module_msgs::KinematicsPose msg )
{
  kinematics_pose_msg_pub_.publish( msg );

  cout <<"Send Kinematics Pose Msg" << endl;
}

void OriginalManipulation::sendIniPoseMsg( std_msgs::String msg )
{
  ini_pose_msg_pub_.publish ( msg );

  cout << "Go to Manipulator Initial Pose" << endl;
}

void OriginalManipulation::sendSetModeMsg( std_msgs::String msg )
{
  set_mode_msg_pub_.publish ( msg );

  cout << "Set BaseModule" << endl;
}
