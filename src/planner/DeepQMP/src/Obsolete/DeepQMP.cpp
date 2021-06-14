#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <champ_msgs/ContactsStamped.h>
#include <std_msgs/Int16.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double base[12];
double joints_pos[12];
double joints_vel[12];
double foot_position[4][3];
double foot_reaction[4][3];
int user_command;

int NUM_SAMPLES=0;
std::string DATA_FILE = "/home/mrjohd/Quadrupeds_ws/src/data/csv/Quadruped_data_1000.txt";
std::fstream dataout(DATA_FILE, std::ios::out);

void WriteData(){
  if(dataout.is_open()){
    if(NUM_SAMPLES == 0){
      dataout <<"Num"
              <<","<<"baseX"<<","<<"baseY"<<","<<"baseZ"
              <<","<<"baseR"<<","<<"baseP"<<","<<"baseYaw"
              <<","<<"base_dX"<<","<<"base_dY"<<","<<"base_dZ"
              <<","<<"base_dR"<<","<<"base_dP"<<","<<"base_dYaw"
              <<","<<"LF_HAA"<<","<<"LF_KFE"<<","<<"LF_HFE"
              <<","<<"LH_HAA"<<","<<"LH_KFE"<<","<<"LH_HFE"
              <<","<<"RF_HAA"<<","<<"RF_KFE"<<","<<"RF_HFE"
              <<","<<"RH_HAA"<<","<<"RH_KFE"<<","<<"RH_HFE"
              <<","<<"dLF_HAA"<<","<<"dLF_KFE"<<","<<"dLF_HFE"
              <<","<<"dLH_HAA"<<","<<"dLH_KFE"<<","<<"dLH_HFE"
              <<","<<"dRF_HAA"<<","<<"dRF_KFE"<<","<<"dRF_HFE"
              <<","<<"dRH_HAA"<<","<<"dRH_KFE"<<","<<"dRH_HFE"
              <<","<<"P_LFx"<<","<<"P_LFy"<<","<<"P_LFz"
              <<","<<"P_RFx"<<","<<"P_RFy"<<","<<"P_RFz"
              <<","<<"P_LHx"<<","<<"P_LHy"<<","<<"P_LHz"
              <<","<<"P_RHx"<<","<<"P_RHy"<<","<<"P_RHz"
              <<","<<"F_LFx"<<","<<"F_LFy"<<","<<"F_LFz"
              <<","<<"F_RFx"<<","<<"F_RFy"<<","<<"F_RFz"
              <<","<<"F_LHx"<<","<<"F_LHy"<<","<<"F_LHz"
              <<","<<"F_RHx"<<","<<"F_RHy"<<","<<"F_RHz"
              <<","<<"User_Command"
              <<"\n";
    }

    dataout << NUM_SAMPLES
            <<","<<base[0]<<","<<base[1]<<","<<base[2]
            <<","<<base[3]<<","<<base[4]<<","<<base[5]
            <<","<<base[6]<<","<<base[7]<<","<<base[8]
            <<","<<base[9]<<","<<base[10]<<","<<base[11]
            <<","<<joints_pos[0]<<","<<joints_pos[1]<<","<<joints_pos[2]
            <<","<<joints_pos[3]<<","<<joints_pos[4]<<","<<joints_pos[5]
            <<","<<joints_pos[6]<<","<<joints_pos[7]<<","<<joints_pos[8]
            <<","<<joints_pos[9]<<","<<joints_pos[10]<<","<<joints_pos[11]
            <<","<<joints_vel[0]<<","<<joints_vel[1]<<","<<joints_vel[2]
            <<","<<joints_vel[3]<<","<<joints_vel[4]<<","<<joints_vel[5]
            <<","<<joints_vel[6]<<","<<joints_vel[7]<<","<<joints_vel[8]
            <<","<<joints_vel[9]<<","<<joints_vel[10]<<","<<joints_vel[11]
            <<","<<foot_position[0][0]<<","<<foot_position[0][1]<<","<<foot_position[0][2]
            <<","<<foot_position[1][0]<<","<<foot_position[1][1]<<","<<foot_position[1][2]
            <<","<<foot_position[2][0]<<","<<foot_position[2][1]<<","<<foot_position[2][2]
            <<","<<foot_position[3][0]<<","<<foot_position[3][1]<<","<<foot_position[3][2]
            <<","<<foot_reaction[0][0]<<","<<foot_reaction[0][1]<<","<<foot_reaction[0][2]
            <<","<<foot_reaction[1][0]<<","<<foot_reaction[1][1]<<","<<foot_reaction[1][2]
            <<","<<foot_reaction[2][0]<<","<<foot_reaction[2][1]<<","<<foot_reaction[2][2]
            <<","<<foot_reaction[3][0]<<","<<foot_reaction[3][1]<<","<<foot_reaction[3][2]
            <<","<<user_command
            << "\n";
  }
  NUM_SAMPLES++;
}

void TFCallback(const tf2_msgs::TFMessage &_msg){
  // Base trajectory position, Z
  // base_footprint~baselink (only height difference)
  double height = _msg.transforms[0].transform.translation.z;
  if(height > 0.001)  base[2]=height;
}

void OdomCallback(const nav_msgs::Odometry &_msg){
  // Base trajectory position
  base[0] = _msg.pose.pose.position.x;
  base[1] = _msg.pose.pose.position.y;
  // base[2] = _msg.pose.pose.position.z;

  // Base trajectory orientation
  double quat_x=_msg.pose.pose.orientation.x;
  double quat_y=_msg.pose.pose.orientation.y;
  double quat_z=_msg.pose.pose.orientation.z;
  double quat_w=_msg.pose.pose.orientation.w;
  tf2::Quaternion quat(quat_x,quat_y,quat_z,quat_w);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(base[3],base[4],base[5]);

  // Base trajectory linear velocity
  base[6] = _msg.twist.twist.linear.x;
  base[7] = _msg.twist.twist.linear.y;
  base[8] = _msg.twist.twist.linear.z;

  // Base trajectory angular velocity
  base[9] = _msg.twist.twist.angular.x;
  base[10] = _msg.twist.twist.angular.y;
  base[11] = _msg.twist.twist.angular.z;

  //Save data
  WriteData();
}
void JointStateCallback(const sensor_msgs::JointState &_msg){
  /*
  Joint Order
  - lf_hip_joint
  - lf_lower_leg_joint
  - lf_upper_leg_joint
  - lh_hip_joint
  - lh_lower_leg_joint
  - lh_upper_leg_joint
  - rf_hip_joint
  - rf_lower_leg_joint
  - rf_upper_leg_joint
  - rh_hip_joint
  - rh_lower_leg_joint
  - rh_upper_leg_joint
  */
  for(int i=0;i<12;i++){
    joints_pos[i] = _msg.position[i];
    joints_vel[i] = _msg.velocity[i];
  }

}
void FootPositionCallback(const visualization_msgs::MarkerArray &_msg){
  /*
  Feet Order
  - LF
  - RF
  - LH
  - RH
  */
  for(int foot=0;foot<4;foot++){
    foot_position[_msg.markers[foot].id][0] = _msg.markers[foot].pose.position.x;
    foot_position[_msg.markers[foot].id][1] = _msg.markers[foot].pose.position.y;
    foot_position[_msg.markers[foot].id][2] = _msg.markers[foot].pose.position.z;
  }
}
void FootReactionCallback(const champ_msgs::ContactsStamped &_msg){
  /*
  Feet Order
  - LF
  - RF
  - LH
  - RH
  */
  for(int foot=0;foot<4;foot++){
    // Update when the foot is on the ground.
    if(_msg.contacts[foot] == true){
      // when the foot is on the ground, F~=0
      foot_reaction[foot][0] = _msg.reactions[foot].x;
      foot_reaction[foot][1] = _msg.reactions[foot].y;
      foot_reaction[foot][2] = _msg.reactions[foot].z;
    }
    else{
      // When the foot is not contact, F=0
      foot_reaction[foot][0] = 0;
      foot_reaction[foot][1] = 0;
      foot_reaction[foot][2] = 0;
    }
  }

}
void UserCommandCallback(const std_msgs::Int16 &_msg){
  user_command = _msg.data;
}



int main(int _argc, char **_argv){
  ros::init(_argc, _argv, "data_generator");

  ros::NodeHandle nh;
  ros::Subscriber sub_tf = nh.subscribe("/tf",1,&TFCallback);
  ros::Subscriber sub_Odometry = nh.subscribe("/odom",1,&OdomCallback);
  ros::Subscriber sub_JointStates = nh.subscribe("/joint_states",1,&JointStateCallback);
  ros::Subscriber sub_FootPosition = nh.subscribe("/foot",1,&FootPositionCallback);
  ros::Subscriber sub_FootReaction = nh.subscribe("/foot_contacts",1,&FootReactionCallback);
  ros::Subscriber sub_UserCommand = nh.subscribe("/User_Command",1,&UserCommandCallback);

  // Initial feet reaction - CHAMP robot
  // foot_reaction[0][0] = 0.241382750106;foot_reaction[0][1] = -0.147906133441;foot_reaction[0][2] = 0.0228952463565;
  // foot_reaction[1][0] = 0.192607574288;foot_reaction[1][1] = 0.123793525589;foot_reaction[1][2] = 0.0210262944035;
  // foot_reaction[2][0] = 0.0137002647741;foot_reaction[2][1] = -0.0729600304407;foot_reaction[2][2] = 0.113286486566;
  // foot_reaction[3][0] = 0.0241253211002;foot_reaction[3][1] = 0.146913818915;foot_reaction[3][2] = 0.240234926093;

  while (ros::ok() && NUM_SAMPLES < 10000000){//데이타 10,000,000배!

    ros::spinOnce();
  }
  dataout.close();
  return 0;
}
