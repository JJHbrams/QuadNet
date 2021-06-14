#!/usr/bin/env python2
# python
import numpy as np
# ROS
import rospy
from xpp_msgs.msg import RobotStateCartesian
from xpp_msgs.msg import StateLin3d
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

from tf.transformations import quaternion_from_euler
import time


def publish_state_des(pub, Motion):
    for idx in range(Motion.shape[0]):

        States = RobotStateCartesian()

        States.base.pose.position.x = Motion[idx,0]
        States.base.pose.position.y = Motion[idx,1]
        States.base.pose.position.z = Motion[idx,2]

        roll = Motion[idx,3]
        pitch = Motion[idx,4]
        yaw = Motion[idx,5]
        quar = quaternion_from_euler(roll, pitch, yaw)
        States.base.pose.orientation.x = quar[0]
        States.base.pose.orientation.y = quar[1]
        States.base.pose.orientation.z = quar[2]
        States.base.pose.orientation.w = quar[3]

        States.base.twist.linear.x = Motion[idx,6]
        States.base.twist.linear.y = Motion[idx,7]
        States.base.twist.linear.z = Motion[idx,8]

        States.base.twist.angular.x = Motion[idx,9]
        States.base.twist.angular.y = Motion[idx,10]
        States.base.twist.angular.z = Motion[idx,11]

        foot_position = StateLin3d()
        foot_position.pos.x = Motion[idx,12]
        foot_position.pos.y = Motion[idx,13]
        foot_position.pos.z = Motion[idx,14]
        States.ee_motion.append(foot_position)


        foot_position = StateLin3d()
        foot_position.pos.x = Motion[idx,15]
        foot_position.pos.y = Motion[idx,16]
        foot_position.pos.z = Motion[idx,17]
        States.ee_motion.append(foot_position)


        foot_position = StateLin3d()
        foot_position.pos.x = Motion[idx,18]
        foot_position.pos.y = Motion[idx,19]
        foot_position.pos.z = Motion[idx,20]
        States.ee_motion.append(foot_position)


        foot_position = StateLin3d()
        foot_position.pos.x = Motion[idx,21]
        foot_position.pos.y = Motion[idx,22]
        foot_position.pos.z = Motion[idx,23]
        States.ee_motion.append(foot_position)

        foot_reaction = Point()
        foot_reaction.x = Motion[idx,24]
        foot_reaction.y = Motion[idx,25]
        foot_reaction.z = Motion[idx,26]
        States.ee_forces.append(foot_reaction)

        foot_reaction = Point()
        foot_reaction.x = Motion[idx,27]
        foot_reaction.y = Motion[idx,28]
        foot_reaction.z = Motion[idx,29]
        States.ee_forces.append(foot_reaction)

        foot_reaction = Point()
        foot_reaction.x = Motion[idx,30]
        foot_reaction.y = Motion[idx,31]
        foot_reaction.z = Motion[idx,32]
        States.ee_forces.append(foot_reaction)

        foot_reaction = Point()
        foot_reaction.x = Motion[idx,33]
        foot_reaction.y = Motion[idx,34]
        foot_reaction.z = Motion[idx,35]
        States.ee_forces.append(foot_reaction)

        States.ee_contact.append(bool(Motion[idx,36]))
        States.ee_contact.append(bool(Motion[idx,37]))
        States.ee_contact.append(bool(Motion[idx,38]))
        States.ee_contact.append(bool(Motion[idx,39]))

        # print(States)
        pub.publish(States)

        # delay = 0.04
        delay = 0.001
        time.sleep(delay)
    print('Goal : ',Motion[0,-6:])

def publish_states(pub, Motion):
    States = Float64MultiArray()
    States.data = Motion[:,:-6].T.flatten()
    pub.publish(States)

if __name__=='__main__':
    # Initialize ROS python
    print("Load DeepQMP_TEST!")
    rospy.init_node('DeepQMP', anonymous = False)
    Target = np.load('/home/mrjohd/Quadrupeds_ws/src/data/csv/GenMotion.npy')[0]
    # Target = np.load('/home/mrjohd/Quadrupeds_ws/src/data/csv/OriMotion.npy')[0]

    pub = rospy.Publisher('/xpp/state_des',RobotStateCartesian,queue_size=1)
    pub2 = rospy.Publisher('/InitPath',Float64MultiArray,queue_size=1)

    publish_state_des(pub, Target);
    publish_states(pub2, Target);
    rospy.spin()
