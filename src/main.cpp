#include "ros/ros.h"

int main(int argc, char *argv[]){
  
  ros::init(argc, argv, "learn_play");

  //When the first ros::NodeHandle is created it will call
  //ros::start(), and when the last ros::NodeHandle is destroyed, it
  //will call ros::shutdown().
  ros::NodeHandle nh;

  while (ros::ok()){
    
  }
  
  return 0;
}
