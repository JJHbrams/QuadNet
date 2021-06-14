#!/usr/bin/env python3
# Pytorch
import torch
from torch.autograd import Variable
import torch.nn.functional as F
from torch import nn
# Matplot
import matplotlib.pyplot as plt
# Numpy
import numpy as np
# ROS
import rospy
from tf2_msgs import TFMessage.h>
from nav_msgs import Odometry.h>
from sensor_msgs import JointState.h>
from visualization_msgs import MarkerArray.h
from champ_msgs import ContactsStamped.h
from std_msgs import Int16.h
import tf
# ETC
import time

class Net(nn.Module):

    def __init__(self): #
        super(Net, self).__init__()
        self.lstm1 = nn.LSTM(input_size=input_dim, hidden_size=hidden_dim[0], bias=True)
        self.lstm2 = nn.LSTM(input_size=hidden_dim[0], hidden_size=hidden_dim[1], bias=True)
        dslayer1 = 20
        dslayer2 = 25
        self.LSTM_DenseNet = nn.Sequential(
                                              nn.Linear(in_features=hidden_dim[1], out_features=dslayer1, bias=True),
                                              nn.ReLU(),
                                              nn.Linear(in_features=dslayer1, out_features=dslayer2, bias=True),
                                              nn.ReLU(),
                                              nn.Linear(in_features=dslayer2, out_features=output_dim, bias=True),
                                              nn.ReLU(),
                                            )
        nnlayer1 = 10
        nnlayer2 = 40
        self.MLP_DenseNet = nn.Sequential(
                                              nn.Linear(in_features=input_dim, out_features=nnlayer1, bias=True),
                                              nn.ReLU(),
                                              nn.Linear(in_features=nnlayer1, out_features=nnlayer2, bias=True),
                                              nn.ReLU(),
                                              nn.Linear(in_features=nnlayer2, out_features=output_dim, bias=True),
                                              nn.ReLU(),
                                            )

        self.End = nn.Sequential(
                                              nn.Linear(in_features=output_dim*2, out_features=output_dim, bias=False),
                                              # nn.ReLU(),
                                          )

    def forward(self, x):
        # X : data, HnC : Hiddne and Cell state
        lstm_out1, _ = self.lstm1(X, HnC1)
        lstm_out2, _ = self.lstm2(lstm_out1, HnC2)
        LSTMoutput = self.LSTM_DenseNet(lstm_out2[-1])

        mlpX=torch.cat([X[-1,:,0].view(-1,1), X[-1,:,1].view(-1,1)],dim=1)
        NNoutput = self.MLP_DenseNet(mlpX)

        output_ = self.End(torch.cat([NNoutput.view(-1,outpuXorigint_dim), LSTMoutput.view(-1,output_dim)],dim=1))

        return output_

# Global variables
pub = rospy.Publisher('/Network_estimation',custom_states_msgs,queue_size=1)
model = Net()
TIME=[]
Data=[]

def EstimateState(Input):
    rate = rospy.Rate(500)
    # Eval
    ts=int(round(time.time() * 1000))
    output = model(Input)
    tf=int(round(time.time() * 1000))

    TIME.append(tf-ts)
    JointStates = output[12:25]
    JointStates = JointStates.detach().numpy()
    pub.publish(JointStates)
    return 0;

def TFCallback(_msg):
    height = _msg.transforms[0].transform.translation.z;
    if(height > 0.001):
          Data[2]=height;
    return 0;

def OdomCallback(_msg):
    # // Base trajectory position
    Data[0] = _msg.pose.pose.position.x
    Data[1] = _msg.pose.pose.position.y

    # // Base trajectory orientation
    quaternion=(
        quat_x=_msg.pose.pose.orientation.x
        quat_y=_msg.pose.pose.orientation.y
        quat_z=_msg.pose.pose.orientation.z
        quat_w=_msg.pose.pose.orientation.w
    )
    Euler = tf.transformations.quaternion_from_euler(quaternion)
    Data[3] = Euler[0]
    Data[4] = Euler[1]
    Data[5] = Euler[2]

    # // Base trajectory linear velocity
    Data[6] = _msg.twist.twist.linear.x
    Data[7] = _msg.twist.twist.linear.y
    Data[8] = _msg.twist.twist.linear.z

    # // Base trajectory angular velocity
    Data[9] = _msg.twist.twist.angular.x
    Data[10] = _msg.twist.twist.angular.y
    Data[11] = _msg.twist.twist.angular.z

    EstimateState(Data)
    return 0;

def JointStateCallback(_msg):
    for i in range(12):
        Data[12+i] = _msg.position[i];
        Data[24+i] = _msg.velocity[i];
    return 0;

def FootPositionCallback(_msg):
    for foot in range(4):
        Data[36+4*_msg.markers[foot].id+0] = _msg.markers[foot].pose.position.x
        Data[36+4*_msg.markers[foot].id+1] = _msg.markers[foot].pose.position.y
        Data[36+4*_msg.markers[foot].id)+2] = _msg.markers[foot].pose.position.z
    return 0;

def FootReactionCallback(_msg):
    for foot in range(4):
        # // Update when the foot is on the ground.
        if(_msg.contacts[foot] == true):
          # // when the foot is on the ground, F~=0
          Data[48+4*_msg.markers[foot].id+0] = _msg.reactions[foot].x
          Data[48+4*_msg.markers[foot].id+1] = _msg.reactions[foot].y
          Data[48+4*_msg.markers[foot].id+2] = _msg.reactions[foot].z
        else:
          # // When the foot is not contact, F=0
          Data[48+4*_msg.markers[foot].id+0] = 0
          Data[48+4*_msg.markers[foot].id+1] = 0
          Data[48+4*_msg.markers[foot].id+2] = 0
    return 0;

def UserCommandCallback(_msg):
    Data[60] = _msg.data;
    return 0;

if __name__=='__main__':
    # Initialize ROS python
    print("Load DeepQMP!")
    rospy.init_node('DeepQMP', anonymous = False)
    rospy.Subscriber("/tf", tf2_msgs::TFMessage, TFCallback, queue_size=1)
    rospy.Subscriber("/odom", custom_states_msgs, OdomCallback, queue_size=1)
    rospy.Subscriber("/joint_states", custom_states_msgs, JointStateCallback, queue_size=1)
    rospy.Subscriber("/foot", custom_states_msgs, FootPositionCallback, queue_size=1)
    rospy.Subscriber("/foot_contacts", custom_states_msgs, FootReactionCallback, queue_size=1)
    rospy.Subscriber("/User_Command", custom_states_msgs, UserCommandCallback, queue_size=1)

    # load net.prm
    params = torch.load("src/dynamo_planner/model/net.tar", map_location = "cpu")
    model.load_state_dict(params['model_state_dict'])
    model.eval()

    # callback
    rospy.spin()

    print("\nMaximum Time : ",np.max(TIME),"ms")
    print("Mean Time    : ",np.mean(TIME),"ms")
