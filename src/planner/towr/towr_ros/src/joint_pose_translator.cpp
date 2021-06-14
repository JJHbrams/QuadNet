#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <xpp_msgs/RobotStateJoint.h>


class JointPoseTranslator
{
public:
	JointPoseTranslator();

private:
	void StateCallback(const xpp_msgs::RobotStateJoint& msg);

	std_msgs::Float64 cmd_RF_HAA, cmd_LF_HAA, cmd_RH_HAA, cmd_LH_HAA;
  std_msgs::Float64 cmd_RF_HFE, cmd_LF_HFE, cmd_RH_HFE, cmd_LH_HFE;
  std_msgs::Float64 cmd_RF_KFE, cmd_LF_KFE, cmd_RH_KFE, cmd_LH_KFE;

  ros::NodeHandle nh_;

  ros::Subscriber joint_sub ;

 	ros::Publisher pub_RF_HAA, pub_LF_HAA, pub_RH_HAA, pub_LH_HAA;
	ros::Publisher pub_RF_HFE, pub_LF_HFE, pub_RH_HFE, pub_LH_HFE;
	ros::Publisher pub_RF_KFE, pub_LF_KFE, pub_RH_KFE, pub_LH_KFE;
};

JointPoseTranslator::JointPoseTranslator()
{
	joint_sub = nh_.subscribe("xpp/joint_popi_des", 1, &JointPoseTranslator::StateCallback, this);
	// HAA
	pub_RF_HAA = nh_.advertise<std_msgs::Float64>("/champ/RF_HAA_eff_position_controller/command", 1);//RF
	pub_LF_HAA = nh_.advertise<std_msgs::Float64>("/champ/LF_HAA_eff_position_controller/command", 1);//LF
	pub_RH_HAA = nh_.advertise<std_msgs::Float64>("/champ/RH_HAA_eff_position_controller/command", 1);//RH
	pub_LH_HAA = nh_.advertise<std_msgs::Float64>("/champ/LH_HAA_eff_position_controller/command", 1);//LH
	// HFE
	pub_RF_HFE = nh_.advertise<std_msgs::Float64>("/champ/RF_HFE_eff_position_controller/command", 1);
	pub_LF_HFE = nh_.advertise<std_msgs::Float64>("/champ/LF_HFE_eff_position_controller/command", 1);
	pub_RH_HFE = nh_.advertise<std_msgs::Float64>("/champ/RH_HFE_eff_position_controller/command", 1);
	pub_LH_HFE = nh_.advertise<std_msgs::Float64>("/champ/LH_HFE_eff_position_controller/command", 1);
	// KFE
	pub_RF_KFE = nh_.advertise<std_msgs::Float64>("/champ/RF_KFE_eff_position_controller/command", 1);
	pub_LF_KFE = nh_.advertise<std_msgs::Float64>("/champ/LF_KFE_eff_position_controller/command", 1);
	pub_RH_KFE = nh_.advertise<std_msgs::Float64>("/champ/RH_KFE_eff_position_controller/command", 1);
	pub_LH_KFE = nh_.advertise<std_msgs::Float64>("/champ/LH_KFE_eff_position_controller/command", 1);
}

void JointPoseTranslator::StateCallback(const xpp_msgs::RobotStateJoint& msg)
{

		cmd_LF_HAA.data = msg.joint_state.position.at(0);
  	cmd_LF_HFE.data = msg.joint_state.position.at(1);
  	cmd_LF_KFE.data = msg.joint_state.position.at(2);

  	cmd_RF_HAA.data = msg.joint_state.position.at(3);
  	cmd_RF_HFE.data = msg.joint_state.position.at(4);
  	cmd_RF_KFE.data = msg.joint_state.position.at(5);

  	cmd_LH_HAA.data = msg.joint_state.position.at(6);
  	cmd_LH_HFE.data = msg.joint_state.position.at(7);
  	cmd_LH_KFE.data = msg.joint_state.position.at(8);

  	cmd_RH_HAA.data = msg.joint_state.position.at(9);
  	cmd_RH_HFE.data = msg.joint_state.position.at(10);
  	cmd_RH_KFE.data = msg.joint_state.position.at(11);

		// LF
  	pub_LF_HAA.publish(cmd_LF_HAA);
  	pub_LF_HFE.publish(cmd_LF_HFE);
  	pub_LF_KFE.publish(cmd_LF_KFE);
		// RF
  	pub_RF_HAA.publish(cmd_RF_HAA);
  	pub_RF_HFE.publish(cmd_RF_HFE);
  	pub_RF_KFE.publish(cmd_RF_KFE);
		// LH
  	pub_LH_HAA.publish(cmd_LH_HAA);
  	pub_LH_HFE.publish(cmd_LH_HFE);
  	pub_LH_KFE.publish(cmd_LH_KFE);
		// RH
  	pub_RH_HAA.publish(cmd_RH_HAA);
  	pub_RH_HFE.publish(cmd_RH_HFE);
  	pub_RH_KFE.publish(cmd_RH_KFE);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_pose_translator");
  JointPoseTranslator joint_pose_translator;

  ros::spin();
}
