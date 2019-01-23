#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "OriginalManipulation.hpp"
#include "manipu_h_motion/toRobotiq.h"
#include "manipu_h_motion/get_param.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <ypspur.h>

geometry_msgs::PoseStamped data;
ros::Subscriber sub;
ros::ServiceClient toRobotiqClient;
ros::ServiceClient getParamClient;


int main(int argc,char *argv[]) {
  
  ros::init(argc,argv,"manipulator_h");
  ros::NodeHandle nh;
  //  sub = nh.subscribe("/linemod/objectPosition",1000,cb);
  toRobotiqClient = nh.serviceClient<manipu_h_motion::toRobotiq>("robotiq_move");
  getParamClient  = nh.serviceClient<manipu_h_motion::get_param>("getObjectParam");

  manipu_h_motion::toRobotiq srv;
  srv.request.speed= 20;
  srv.request.force= 10;

  manipu_h_motion::get_param objeSrv;

  // **マニピュレータ(初期形状)**//
  OriginalManipulation manipulator;
  manipulator.SetMode();
  sleep(2);
  manipulator.SetInitPose();
  sleep(5);


  char key_data;
  while(1){
    std::cout << "キーボード入力待ち（'0':cylinder '1':rectangle 'n':終了)\n-->";
    std::cin >> key_data;
    if(key_data == '0' || key_data == '1'){ 
      
      if(key_data == '0'){
	objeSrv.request.object_num = 0;	
      }else{
	objeSrv.request.object_num = 1;
      }

      if(getParamClient.call(objeSrv)){
       	std::cout<<"x:"<<objeSrv.response.x<<" y:"<<objeSrv.response.y<<" z:"<<0.015/*objeSrv.response.z*/<<std::endl;
	std::cout<<"r:"<<objeSrv.response.roll<<" p:"<<objeSrv.response.pitch<<" y:"<<objeSrv.response.yaw<<std::endl;
      }else{
	std::cout<<"パラメータを取得できません" <<std::endl;
      }

      std::cout << "上記の場所へ移動しますか？  y:する n:しない\n-->";
      std::cin >> key_data;
      if(key_data == 'y'){
	manipulator.SetKinematicsPose(objeSrv.response.x-0.082,objeSrv.response.y , 0.26 , 0.0 , 65 , 0);
	sleep(10);
	srv.request.open = 255;
	if(toRobotiqClient.call(srv)){
	  std::cout << srv.response.end_num;
	}else{
	  std::cout <<"robotiqで問題が発生しました"<<std::endl;
	}
	break;
      }else{;}

    }else if(key_data == 'n'){
      std::cout << "動作を終了します" << std::endl;
      break;

    }else{
      std::cout << "もう一度入力してください\n-->";
    }
  }

  while(1){
    std::cout << "初期位置に戻るには'b'を押してください\n-->";
    std::cin >> key_data;
    if(key_data == 'b'){
      break;
    }   
  }
  manipulator.SetInitPose();
  sleep(5);

  while(1){
    std::cout << "h:ハンドを開く\n-->";
    std::cin >> key_data;
    if(key_data == 'h'){
      srv.request.open = 0;
      if(toRobotiqClient.call(srv)){
	  std::cout << srv.response.end_num;
      }else{
	  std::cout <<"robotiqで問題が発生しました"<<std::endl;
      }
      break;
    }else{;}
  }
  return 0;
}
