// ROS
#include "ros/ros.h"
#include <ros/callback_queue.h>
// C++
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <random>
#include <boost/algorithm/string.hpp>
#include <string>
// Messages
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <towr_data_generator/towrGen_msgs.h>
#include <visualization_msgs/MarkerArray.h>
// Transform - ROS
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//데이타 10,000,000배!
#define MAX_SMAPLE_NUM 1000000

double terrain[200][200][3]={0};
double x_min=-5.0;
double y_min=-5.0;
double x_max=5.0;
double y_max=5.0;
double dxy=0.1;
int TerrainID=0;
bool initial_path=true;

std::string DATA_FILE = "/home/mrjohd/Quadrupeds_ws/src/data/csv/Terrain_data.txt";
std::fstream dataout(DATA_FILE, std::ios::out);

void WriteData(){
  if(dataout.is_open()){
    if(initial_path){
      for(int i=0; i<300; i++){
        if(i==299)  dataout << std::to_string(i);
        else dataout << std::to_string(i) << ",";
      }
      dataout << "\n";
      initial_path = false;
    }

    else{
      for(int iy=0; iy<100; iy++){
          for(int ix=0; ix<100; ix++){
            if(ix==99)  dataout << terrain[ix][iy][0] << "," << terrain[ix][iy][1] << "," << terrain[ix][iy][2];
            else dataout << terrain[ix][iy][0] << "," << terrain[ix][iy][1] << "," << terrain[ix][iy][2] << ", ";
          }
          dataout << "\n";
      }
    }
  }
}

void XppTerrainCallback(const visualization_msgs::MarkerArray& _msg){
  int idx = 0;
  int idy = 0;

  std::cout << "Reading..." << std::endl;
  for(int i=0;i<10201;i++){
    idx = (_msg.markers[i].pose.position.x-x_min)/dxy;
    idy = (_msg.markers[i].pose.position.y-y_min)/dxy;
    // std::cout << idx << ", " << idy << std::endl;
    if(terrain[idx][idy][2] == 0){
      terrain[idx][idy][0] = _msg.markers[i].pose.position.x;
      terrain[idx][idy][1] = _msg.markers[i].pose.position.y;
      terrain[idx][idy][2] = _msg.markers[i].pose.position.z;
    }
    else
      continue;
  }
  std::cout << "Writing..." << std::endl;
  //Save data
  WriteData();
}

int main(int _argc, char **_argv){
  ros::init(_argc, _argv, "towr_terrain_data_generator");

  ros::NodeHandle nh;
  ros::Subscriber sub_xpp_terrain = nh.subscribe("/xpp/terrain",1,&XppTerrainCallback);

  // ros::spin();
  while (ros::ok()){
    ros::spinOnce();
  }
  dataout.close();
  return 0;
}
