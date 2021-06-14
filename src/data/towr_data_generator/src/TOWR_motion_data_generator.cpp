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

double LinX, LinY, AngX, AngY, AngZ;
double base[12];
double foot_position[4][3];
double foot_reaction[4][3];
bool foot_contact[4];
bool initial_path = true;
bool NoForces=true;
bool opt_flag=true;

int NUM_SAMPLES=0;
int NUM_PATH=-1;
std::string DATA_FILE = "/home/mrjohd/Quadrupeds_ws/src/data/csv/Hound_data.txt";
std::fstream dataout(DATA_FILE, std::ios::out);

void WriteData(){
  if(base[0]==0 && base[1]==0 && base[2]==0.5 &&
     base[3]==0 && base[4]==0 && base[5]==0 &&
     foot_reaction[0][0]==0 && foot_reaction[0][1]==0 && foot_reaction[0][2]==0 &&
     foot_reaction[1][0]==0 && foot_reaction[1][1]==0 && foot_reaction[1][2]==0 &&
     foot_reaction[2][0]==0 && foot_reaction[2][1]==0 && foot_reaction[2][2]==0 &&
     foot_reaction[3][0]==0 && foot_reaction[3][1]==0 && foot_reaction[3][2]==0   ){
    // Initial pose
    NUM_PATH++;
    NUM_SAMPLES=0;
    NoForces=true;
  }
  if(dataout.is_open()){
    if(initial_path){
      dataout <<"PathNum"
              <<", "<<"NodeNum"
              <<", "<<"baseX"<<","<<"baseY"<<","<<"baseZ"
              <<", "<<"baseR"<<","<<"baseP"<<","<<"baseYaw"
              <<", "<<"base_dX"<<","<<"base_dY"<<","<<"base_dZ"
              <<", "<<"base_dR"<<","<<"base_dP"<<","<<"base_dYaw"
              <<", "<<"P_LFx"<<","<<"P_LFy"<<","<<"P_LFz"
              <<", "<<"P_RFx"<<","<<"P_RFy"<<","<<"P_RFz"
              <<", "<<"P_LHx"<<","<<"P_LHy"<<","<<"P_LHz"
              <<", "<<"P_RHx"<<","<<"P_RHy"<<","<<"P_RHz"
              <<", "<<"F_LFx"<<","<<"F_LFy"<<","<<"F_LFz"
              <<", "<<"F_RFx"<<","<<"F_RFy"<<","<<"F_RFz"
              <<", "<<"F_LHx"<<","<<"F_LHy"<<","<<"F_LHz"
              <<", "<<"F_RHx"<<","<<"F_RHy"<<","<<"F_RHz"
              <<", "<<"LF"<<","<<"RF"<<","<<"LH"<<","<<"RH"
              <<"\n";

      initial_path = false;
    }

    if(initial_path==false && NoForces== false){
      dataout << NUM_PATH
              <<", "<< NUM_SAMPLES
              <<", "<<base[0]<<","<<base[1]<<","<<base[2]
              <<", "<<base[3]<<","<<base[4]<<","<<base[5]
              <<", "<<base[6]<<","<<base[7]<<","<<base[8]
              <<", "<<base[9]<<","<<base[10]<<","<<base[11]
              <<", "<<foot_position[0][0]<<","<<foot_position[0][1]<<","<<foot_position[0][2]
              <<", "<<foot_position[1][0]<<","<<foot_position[1][1]<<","<<foot_position[1][2]
              <<", "<<foot_position[2][0]<<","<<foot_position[2][1]<<","<<foot_position[2][2]
              <<", "<<foot_position[3][0]<<","<<foot_position[3][1]<<","<<foot_position[3][2]
              <<", "<<foot_reaction[0][0]<<","<<foot_reaction[0][1]<<","<<foot_reaction[0][2]
              <<", "<<foot_reaction[1][0]<<","<<foot_reaction[1][1]<<","<<foot_reaction[1][2]
              <<", "<<foot_reaction[2][0]<<","<<foot_reaction[2][1]<<","<<foot_reaction[2][2]
              <<", "<<foot_reaction[3][0]<<","<<foot_reaction[3][1]<<","<<foot_reaction[3][2]
              <<", "<<foot_contact[0]<<","<<foot_contact[1]<<","<<foot_contact[2]<<","<<foot_contact[3]
              << "\n";
    }
  }
  NUM_SAMPLES++;
  NoForces=false;
}

void XppStateCallback(const xpp_msgs::RobotStateCartesian& _msg){
  // Base trajectory position
  base[0] = _msg.base.pose.position.x;
  base[1] = _msg.base.pose.position.y;
  base[2] = _msg.base.pose.position.z;

  // Base trajectory orientation
  double quat_x=_msg.base.pose.orientation.x;
  double quat_y=_msg.base.pose.orientation.y;
  double quat_z=_msg.base.pose.orientation.z;
  double quat_w=_msg.base.pose.orientation.w;
  tf2::Quaternion quat(quat_x,quat_y,quat_z,quat_w);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(base[3],base[4],base[5]);

  // Base trajectory linear velocity
  base[6] = _msg.base.twist.linear.x;
  base[7] = _msg.base.twist.linear.y;
  base[8] = _msg.base.twist.linear.z;

  // Base trajectory angular velocity
  base[9] = _msg.base.twist.angular.x;
  base[10] = _msg.base.twist.angular.y;
  base[11] = _msg.base.twist.angular.z;

  /*
  Feet Order
  - LF
  - RF
  - LH
  - RH
  */
  // End Effector pose
  for(int foot=0;foot<4;foot++){
    foot_position[foot][0] = _msg.ee_motion[foot].pos.x;
    foot_position[foot][1] = _msg.ee_motion[foot].pos.y;
    foot_position[foot][2] = _msg.ee_motion[foot].pos.z;
  }

  // End effector reaction
  for(int foot=0;foot<4;foot++){
    foot_reaction[foot][0] = _msg.ee_forces[foot].x;
    foot_reaction[foot][1] = _msg.ee_forces[foot].y;
    foot_reaction[foot][2] = _msg.ee_forces[foot].z;
  }

  // End effector contact
  for(int foot=0;foot<4;foot++){
    foot_contact[foot] = _msg.ee_contact[foot];
  }

  //Save data
  WriteData();
}

std::mt19937 generator;
double RAND_VAL(double MIN, double MAX, char MODE){
  switch(MODE){
    case 'u':{
      // Uniform distribution
      std::uniform_real_distribution<double> uniform(MIN, MAX);
      srand(uniform(generator)*(time(NULL)));
      break;
    }

    case 'n':{
      // Normaldistribution
      double mean = (MAX+MIN)/2;
      double stddev  = 1;
      std::normal_distribution<double> normal(mean, stddev);
      srand(normal(generator)*(time(NULL)));
      break;
    }

  }

  // std::cerr << "Normal: " << normal(generator) << std::endl;
  double RES = 1E4;
  int DIV = static_cast<int>((MAX-MIN)*RES+1);
  double SHIFT = ((MAX-MIN)/2.)*RES;
  double MEAN = (MAX+MIN)/2*RES;

  return (rand()%DIV - SHIFT + MEAN)/RES;
}

void PublishTOWRcommand(ros::Publisher pub){
  towr_data_generator::towrGen_msgs msg;

  // LinX = RAND_VAL(-2.5,2.5,'n');
  LinX = RAND_VAL(0.1,2.5,'n');
  LinY = RAND_VAL(-2.5,2.5,'n');
  AngX = RAND_VAL(-M_PI,M_PI,'n');  AngY = RAND_VAL(-M_PI,M_PI,'n');
  // AngZ = RAND_VAL(-M_PI,M_PI,'n');
  AngZ = RAND_VAL(-M_PI/3,M_PI/3,'n');
  if(AngX == -M_PI)  AngX = M_PI;  if(AngY == -M_PI)  AngY = M_PI;  if(AngZ == -M_PI)  AngZ = M_PI;
  double total_duration = 2.0;

  msg.goalX = LinX;
  msg.goalY = LinY;
  msg.goalYAW = AngZ;
  msg.duration = total_duration;
  // msg.terrain                  = terrain_;

  pub.publish(msg);
}

int main(int _argc, char **_argv){
  ros::init(_argc, _argv, "towr_data_generator");

  ros::NodeHandle nh;
  ros::Subscriber sub_xpp_state = nh.subscribe("/xpp/state_des",1,&XppStateCallback);
  ros::Publisher pub_xpp_state = nh.advertise<towr_data_generator::towrGen_msgs>("/TOWR_command",1);

  double rate = 100;

  // ros::spin();
  while (ros::ok() && NUM_PATH < MAX_SMAPLE_NUM){
    PublishTOWRcommand(pub_xpp_state);
    ros::spinOnce();
    // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1/rate));
  }
  dataout.close();
  return 0;
}
