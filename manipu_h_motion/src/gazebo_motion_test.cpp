#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "OriginalManipulation.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <ypspur.h>


const float PI = 3.141592653;

int main(int argv,char *argc[]) {
  
  ros::init(argv,argc,"manipulator_h_abe_test");
  ros::NodeHandle nh;
  
  // **マニピュレータ(初期形状)**//
  OriginalManipulation manipulator;
  sleep(1);//←このsleepがめちゃくちゃ大事！！！！！(guiのsetModeいらなくなった)
  manipulator.SetMode();
  sleep(3);
  std::cout << "   SetInitPose go" << std::endl;
  manipulator.SetInitPose();
  sleep(5);
  
  //**1回目動作**//
  double manipulator_x,manipulator_y,manipulator_z;
  manipulator_x = 0.1;
  manipulator_y = 0.0;
  manipulator_z = 0.564+0.01;
  std::cout << "   1 position go" << std::endl;
  manipulator.SetKinematicsPose(manipulator_x,manipulator_y,manipulator_z,0.0,0.040,0.0);
  sleep(5);
  
  //**2回目動作**//
  // std::cout << "   2 position go" << std::endl;  
  kpose manipu_h;
  manipu_h.x_ = 0.4;
  manipu_h.y_ = 0.0;
  manipu_h.z_ = 0.3;
  manipu_h.roll_=0.0;
  manipu_h.pitch_=0.0;
  manipu_h.yaw_=0.0;
  manipulator.SetKinematicsPose(manipu_h);
  sleep(5);
  




  // manipulator_x = 0.3;
  // manipulator_y = 0.0;
  // manipulator_z = 0.3;
  
  // manipulator.SetKinematicsPose(manipulator_x,manipulator_y,manipulator_z,0.0,90,0);
  // sleep(5);
  


  std::cout << "   motin finish" << std::endl;
  std::cout << "    go SetInitPose" << std::endl;
  manipulator.SetInitPose();
  sleep(5);


  return 0;
}
