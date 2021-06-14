#!/usr/bin/env python3
# python
import numpy as np
import torch
from torch.autograd import Variable
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms
from torchvision.utils import save_image

from torch.utils.data import DataLoader
from torch.utils.data import Dataset

import time
import copy as cp
import itertools
# ROS
import rospy
from xpp_msgs.msg import RobotStateCartesian
from xpp_msgs.msg import StateLin3d
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from towr_ros.msg import TowrCommand
from visualization_msgs.msg import MarkerArray

import time
import matplotlib.pyplot as plt

# cuda = True if torch.cuda.is_available() else False
cuda = False
Tensor = torch.cuda.FloatTensor if cuda else torch.FloatTensor

MaxSampleNum = 201
Path_dim = 40
z_dim = 128
dt=2
FEET = [0,3,6,9]
Xmin=-5.;    Ymin=-5.;    res=0.1;

terrain_flag=False

class GenTerrainDataset(Dataset):

  def __init__(self, TerrainData, transform=None):
      self.data = TerrainData.float()
      self.transform = transform

  def __getitem__(self, index):
      data = self.data[index]
      if self.transform:
            data = self.transform(data)

      return index, data

  def __len__(self):
      return len(self.data)

def cutoff(X, sv=0.5):
    return torch.tensor(X>sv, dtype=torch.float)

class swish(nn.Module):
  def __init__(self):
    super(swish, self).__init__()

  def forward(self, X):
    return X * torch.sigmoid(X)

class mish(nn.Module):
  def __init__(self):
    super(mish, self).__init__()

  def forward(self, X):
    return X * torch.tanh(F.softplus(X))

class View(nn.Module):
    def __init__(self, size):
        super(View, self).__init__()
        self.size = size

    def forward(self, tensor):
        return tensor.view(self.size)

def Getheight(inX, inY, Terrain):
  thres = 0.1
  desX = inX
  desY = inY
  ycor=torch.where(abs(Terrain[0]-desX)<thres)
  ycor = ycor[-1].max()
  xcor=torch.where(abs(Terrain[1,:,ycor]-desY)<thres)
  xcor=xcor[0].max()
  return Terrain[2,xcor,ycor]

def FinalState(Init, Goals, Terr):
  # Calculate final states
  roll = torch.zeros((Goals.shape[0],1)).type(Tensor)
  pitch = torch.zeros((Goals.shape[0],1)).type(Tensor)
  yaw = Goals[:,-1].view((Goals.shape[0],1))
  tensor_0 = torch.zeros((Goals.shape[0],1)).type(Tensor)
  tensor_1 = torch.ones((Goals.shape[0],1)).type(Tensor)

  RX = torch.stack([
                  torch.stack([tensor_1, tensor_0, tensor_0]),
                  torch.stack([tensor_0, torch.cos(roll), -torch.sin(roll)]),
                  torch.stack([tensor_0, torch.sin(roll), torch.cos(roll)])]).permute(3,2,0,1).view(-1,3,3)
  RY = torch.stack([
                  torch.stack([torch.cos(pitch), tensor_0, torch.sin(pitch)]),
                  torch.stack([tensor_0, tensor_1, tensor_0]),
                  torch.stack([-torch.sin(pitch), tensor_0, torch.cos(pitch)])]).permute(3,2,0,1).view(-1,3,3)
  RZ = torch.stack([
                  torch.stack([torch.cos(yaw), -torch.sin(yaw), tensor_0]),
                  torch.stack([torch.sin(yaw), torch.cos(yaw), tensor_0]),
                  torch.stack([tensor_0, tensor_0, tensor_1])]).permute(3,2,0,1).view(-1,3,3)

  R = torch.bmm(RZ.view(-1,3,3), RY.view(-1,3,3))
  wRb = torch.bmm(R, RX.view(-1,3,3))

  FinalStance=0
  FinalBase = cp.deepcopy(Goals[:,:3].view(-1,3,1)); FinalBase[:,-1,0]-=(Goals[:,2]-Getheight(Goals[:,0],Goals[:,1],Terr))
  NominalStance = Init[:,12:24]
  for feet in FEET:
    tfStance = torch.bmm(wRb, NominalStance[:,feet:feet+3].view(-1,3,1))+FinalBase
    if feet==0:
      FinalStance = tfStance
    else:
      FinalStance=torch.cat([FinalStance, tfStance], dim=1)
  FinalStance=FinalStance.view(-1,12)
  FinalStates = []
  FinalStates = torch.cat([Goals,torch.zeros((Goals.shape[0],6)).type(Tensor)],dim=1)
  FinalStates = torch.cat([FinalStates, FinalStance], dim=1)
  FinalStates = torch.cat([FinalStates, torch.zeros((Goals.shape[0],12)).type(Tensor)], dim=1)
  FinalStates = torch.cat([FinalStates, torch.ones((Goals.shape[0],4)).type(Tensor)], dim=1)

  return FinalStates

def interNodes(Nodes, batch_size, seq_size=MaxSampleNum//dt, type='bilinear'):
  data=Nodes.view(1,batch_size,seq_size,Path_dim).permute(1,3,0,2)

  InterState=F.interpolate(data[:,:-4,:,:], [1,MaxSampleNum], mode=type) # 4D 1-dim height form
  InterContact=F.interpolate(data[:,-4:,:,:], [1,MaxSampleNum], mode='nearest') # 4D 1-dim height form
  Inter = torch.cat([InterState, InterContact], dim=1)

  return Inter.permute(2,0,3,1).view(batch_size,MaxSampleNum,Path_dim)

def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

TOWRmsg = [TowrCommand()]
Goal=torch.zeros(1,6)
Terrain_ = 0
speed=[]
speed.append(1)
path_flag = []
path_flag.append(False)
TIME=[]
TIME2=[]
# minTerr=-0.4
minTerr=0
def TerrainCallback(msg):
    terrainMAP = torch.zeros((1,3,100,100))
    for i in range(10000):
        idx = int((msg.markers[i].pose.position.x-Xmin)/res);
        idy = int((msg.markers[i].pose.position.y-Ymin)/res);
        terrainMAP[0,0,idy,idx]=msg.markers[i].pose.position.x;
        terrainMAP[0,1,idy,idx]=msg.markers[i].pose.position.y;
        terrainMAP[0,2,idy,idx]=msg.markers[i].pose.position.z*2;

    # Gap...
    global minTerr
    minTerr = terrainMAP[0,2].min()
    terrainMAP[0,2] -= terrainMAP[0,2].min()
    print("TerrainCallback : ",terrainMAP[0,2].max())
    print("TerrainCallback : ",terrainMAP[0,2].min())

    dataset = GenTerrainDataset(terrainMAP, transform=transform)
    terrain_dataloader = DataLoader(dataset,batch_size=1,shuffle=False)
    global TERRAIN
    _, TERRAIN = next(iter(terrain_dataloader))
    TERRAIN[0,0]=torch.linspace(-5,5,64).view(1,-1).repeat(64,1)
    TERRAIN[0,1]=torch.linspace(-5,5,64).view(-1,1).repeat(1,64)
    del dataset,terrain_dataloader

    # print("TerrainCallback : ",terrain_flag)

def TOWRCallback(msg):
    TOWRmsg.append(msg)
    Goal[0,0] = msg.goal_lin.pos.x
    Goal[0,1] = msg.goal_lin.pos.y
    Goal[0,2] = msg.goal_lin.pos.z + 0.5
    Goal[0,3] = msg.goal_ang.pos.x
    Goal[0,4] = msg.goal_ang.pos.y
    Goal[0,5] = msg.goal_ang.pos.z
    Terrain_ = msg.terrain
    speed.append(msg.replay_speed)
    path_flag.append(msg.optimize or msg.play_initialization)
    # print('Goal pose : ',Goal)

def publish_state_des(pub, Motion):

    for idx in range(Motion.shape[0]):

        States = RobotStateCartesian()

        States.base.pose.position.x = Motion[idx,0]
        States.base.pose.position.y = Motion[idx,1]
        States.base.pose.position.z = Motion[idx,2]

        roll = Motion[idx,3]
        pitch = Motion[idx,4]
        yaw = Motion[idx,5]
        quar = euler_to_quaternion(roll, pitch, yaw)
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
        delay = 0.01/speed[-1]
        time.sleep(delay)
    print('Last motion : ',Motion[-1,:6])

class TeerainEncoder(nn.Module):
  def __init__(self):
        super(TeerainEncoder, self).__init__()
        latent_dim = z_dim
        self.latent_dim = latent_dim
        self.im_channels = in_channels = 1

        modules = []
        hidden_dims = [32, 64, 64, 128, 256]
        self.hidden_dims = hidden_dims.copy()

        # Build Encoder
        for h_dim in hidden_dims:
            modules.append(
                nn.Sequential(
                    nn.Conv2d(in_channels, out_channels=h_dim,
                              kernel_size= 3, stride= 2, padding  = 1),
                    nn.BatchNorm2d(h_dim),
                    nn.LeakyReLU())
            )
            in_channels = h_dim

        self.encoder = nn.Sequential(*modules)
        self.LatentSpace = nn.Linear(hidden_dims[-1]*4, latent_dim)

        # Build Decoder
        modules = []
        self.decoder_input = nn.Linear(latent_dim, hidden_dims[-1]*4)
        hidden_dims.reverse()
        for i in range(len(hidden_dims) - 1):
            modules.append(
                nn.Sequential(
                    nn.ConvTranspose2d(hidden_dims[i],
                                       hidden_dims[i + 1],
                                       kernel_size=3,
                                       stride = 2,
                                       padding=1,
                                       output_padding=1),
                    nn.BatchNorm2d(hidden_dims[i + 1]),
                    nn.LeakyReLU())
            )
        self.decoder = nn.Sequential(*modules)
        self.final_layer = nn.Sequential(
                                        nn.ConvTranspose2d(hidden_dims[-1],
                                                            hidden_dims[-1],
                                                            kernel_size=3,
                                                            stride=2,
                                                            padding=1,
                                                            output_padding=1),
                                        nn.BatchNorm2d(hidden_dims[-1]),
                                        nn.LeakyReLU(),
                                        nn.Conv2d(hidden_dims[-1], out_channels= self.im_channels,
                                                  kernel_size= 3, padding= 1),
                                        nn.Tanh())

  def encode(self, input):
      result = self.encoder(input)
      result = torch.flatten(result, start_dim=1)
      latent = self.LatentSpace(result)

      return latent

  def decode(self, z):
      result = self.decoder_input(z)
      result = result.view(-1, self.hidden_dims[-1], 2, 2)
      result = self.decoder(result)
      result = self.final_layer(result)

      return result

  def forward(self, input):
      z = self.encode(input)
      recon = self.decode(z)
      return  [recon, input, z]

class DeepQMP(nn.Module): # torch.nn.Module을 상속받는 파이썬 클래스
    def __init__(self):
        super(DeepQMP, self).__init__()
        input_dim=40
        output_dim=40
        self.output_dim = output_dim
        goal_dim=6

                # CNN
        ch1 = 24
        self.ConvReal = nn.Sequential(
                                              nn.Conv2d(1, ch1, (3, 1), stride=(3,1)),
                                              nn.BatchNorm2d(ch1),
                                              # nn.LeakyReLU(alpha, inplace=True),
                                              mish(),
                                              # 14 x 24
                                  )
        self.ConvBin = nn.Sequential(
                                              nn.Conv2d(1, ch1, (4, 1), stride=1),
                                              nn.BatchNorm2d(ch1),
                                              mish(),
                                              # 1 x 24
                                  )
        self.ConvEnv = nn.Sequential(
                                              nn.Conv2d(1, ch1, (5, 1), stride=(3,1)),
                                              nn.BatchNorm2d(ch1),
                                              mish(),
                                              # 42 x 24
                                    )
        ch2 = 1
        ch3 = 8
        ch4 = 16
        ch5 = 32
        featW = 13
        featH = 2
        sub_input_dim = featW * featH * ch5
        self.StateFeatures = nn.Sequential(
                                              nn.Conv2d(1, ch2, (3, 3), stride=3),
                                              nn.BatchNorm2d(ch2),
                                              mish(),
                                              nn.Conv2d(ch2, ch3, (3, 3), stride=1),
                                              nn.BatchNorm2d(ch3),
                                              mish(),
                                              nn.Conv2d(ch3, ch4, (3, 3), stride=1),
                                              nn.BatchNorm2d(ch4),
                                              mish(),
                                              nn.Conv2d(ch4, ch5, (3, 3), stride=1),
                                              nn.BatchNorm2d(ch5),
                                              # mish(),
                                              nn.Flatten(),
                                              nn.Linear(sub_input_dim, output_dim, bias=True),
                                              nn.BatchNorm1d(output_dim),
                                              mish(),
                                            )

        dslayer1 = 64
        dslayer2 = 128
        dslayer3 = 256
        self.CNN_DenseNet = nn.Sequential(
                                                nn.Linear(output_dim, dslayer1, bias=True),
                                                nn.BatchNorm1d(dslayer1),
                                                mish(),

                                                nn.Linear(dslayer1, dslayer2, bias=True),
                                                nn.BatchNorm1d(dslayer2),
                                                mish(),

                                                nn.Linear(dslayer2, dslayer3, bias=True),
                                                nn.BatchNorm1d(dslayer3),
                                                mish(),
                                            )

        # MLP
        self.PathLen = (MaxSampleNum-1)//dt
        self.output_dim = output_dim
        self.GenPath = nn.Sequential(
                                                nn.Linear(dslayer3, self.PathLen*output_dim),
                                      )

    def forward(self, X, Terrain):
        # HnC : Hiddne and Cell state
        # X     = (seq, batch, dim)
        # CnnX  = (batch, ch, dim, seq)
        # lstmX = (seq, batch, dim)
        # mlpX  = (batch, dim)

        # CNN - State features
        CnnX = torch.cat([X[:,:-10], X[:,-6:]], dim=1)
        CnnX = CnnX.reshape(CnnX.shape[0], 1, CnnX.shape[1], 1)
        cnnReal_out = self.ConvReal(CnnX)

        CnnX = X[:,-10:-6]
        CnnX = CnnX.reshape(CnnX.shape[0], 1, CnnX.shape[1], 1)
        cnnBin_out = self.ConvBin(CnnX)

        CnnX = Terrain
        CnnX = CnnX.reshape(CnnX.shape[0], 1, CnnX.shape[1], 1)
        cnnEnv_out = self.ConvEnv(CnnX)

        CnnX = torch.cat([cnnReal_out, cnnBin_out], dim=2)
        CnnX = torch.cat([CnnX, cnnEnv_out], dim=2)
        CONVoutput = self.StateFeatures(CnnX.permute(0,3,1,2))

        # MLP
        mlpX = CONVoutput# + X[:,:-6]
        MLPout = self.CNN_DenseNet(mlpX)

        # Generate path
        FullPath = self.GenPath(MLPout).view(-1,self.PathLen,self.output_dim)
        FullPath[:,:,-4:] = torch.sigmoid(FullPath[:,:,-4:])
        return FullPath

def GenVar(Motions):
    Variables = []

    NUMbase = 10
    baselin = torch.cat([Motions[:,:3], Motions[:,6:9]], dim=1)
    BaseMotion_lin = baselin
    BaseMotion_lin = BaseMotion_lin[0::NUMbase,:]
    # BaseMotion_lin = torch.cat([BaseMotion_lin, baselin[-1,:].view(1,6)],dim=0)
    BaseMotion_lin = BaseMotion_lin.flatten().tolist()
    Variables += BaseMotion_lin
    # print(np.shape(BaseMotion_lin))

    baseang = torch.cat([Motions[:,3:6], Motions[:,9:12]], dim=1)
    BaseMotion_ang = baseang
    BaseMotion_ang = BaseMotion_ang[0::NUMbase,:]
    # BaseMotion_ang = torch.cat([BaseMotion_ang, baseang[-1,:].view(1,6)],dim=0)
    BaseMotion_ang = BaseMotion_ang.flatten().tolist()
    Variables += BaseMotion_ang
    # print(np.shape(BaseMotion_ang))

    EEMotion_ = Motions[:,12:24]
    for feet in FEET:
        NUMswing=4
        EEMotion = EEMotion_[0::25,:]
        if feet == 0 or feet == 9:
            NUMswing=3
            EEMotion = EEMotion_[0::33,:]

        eeMotion = EEMotion[:,feet:feet+3]
        front=torch.cat([eeMotion[1::2,0].view(NUMswing,1), torch.zeros((NUMswing,1))], dim=1)
        rear=torch.cat([torch.zeros((NUMswing,1)), eeMotion[1::2,-1].view(NUMswing,1)], dim=1)
        EEmotionSwing = torch.cat([front, eeMotion[1::2,1].view(NUMswing,1)], dim=1)
        EEmotionSwing = torch.cat([EEmotionSwing, rear], dim=1)
        EEmotionSwing = EEmotionSwing.tolist()
        eeMotion = eeMotion.tolist()
        eeMotion[1::2] = EEmotionSwing
        eeMotion=list(itertools.chain.from_iterable(eeMotion))
        # print(np.shape(eeMotion))
        Variables += eeMotion

    EEForce_ = Motions[:,24:36]
    for feet in FEET:
        NUMforces=12
        EEForce = EEForce_[0::18,:]
        if feet == 0 or feet == 9:
            NUMforces=10
            EEForce = EEForce_[0::22,:]

        eeForce = EEForce[:,feet:feet+3]
        eeForceSam = torch.cat([eeForce[:,0].view(NUMforces,1), torch.zeros((NUMforces,1))], dim=1)
        eeForceSam = torch.cat([eeForceSam, eeForce[:,1].view(NUMforces,1)], dim=1)
        eeForceSam = torch.cat([eeForceSam, torch.zeros((NUMforces,1))], dim=1)
        eeForceSam = torch.cat([eeForceSam, eeForce[:,2].view(NUMforces,1)], dim=1)
        eeForceSam = torch.cat([eeForceSam, torch.zeros((NUMforces,1))], dim=1)
        eeForce = eeForceSam.tolist()
        eeForce=list(itertools.chain.from_iterable(eeForce))
        # print(np.shape(eeForce))
        Variables += eeForce

    print("Initial solution length : ",np.shape(Variables))
    return Variables

def EstimatePath(Encoder, Planner, Init, terr, pub):
    '''
        When make a motion for a gap, we need to compensate the (-height)....
    '''
    print('Generating Path...')
    Goal[0,2] += Getheight(Goal[:,0],Goal[:,1],terr[0])+minTerr
    print("Goal : ",Goal)
    XnG = torch.cat([Init, Goal],dim=1).type(Tensor)
    terrain = terr[:,2]#/terr[:,2].max()
    ts=int(round(time.time() * 1000))
    Env = Encoder(terrain.view(-1,1,64,64))[-1]
    # ts=int(round(time.time() * 1000))
    ts2=int(round(time.time() * 1000))
    Motion = Planner(XnG,Env).detach()
    # Motion[0,:,-16:-4] = (Motion[0,:,-16:-4])*100

    # Interpolate
    seq_size = (MaxSampleNum-1)//dt
    Nodes = cp.deepcopy(Motion)
    Nodes[:,0] = Init
    # Nodes[:,-1] = FinalState(Init, Goal, terr[0])
    # Nodes[:,-1] = Nodes[:,-2]
    # print(FinalState(Init, Goal, terr[0]))
    Predict = interNodes(Motion,1)
    Motion = Predict
    Motion[0,:,-4:] = cutoff(Motion[0,:,-4:])



    Motion[0,:,-16:-4] = (Motion[0,:,-16:-4])*200
    for feet in FEET:
        # Revise feet position z
        feetX=Motion[0,:,12+feet].view(-1,1);   idx=((feetX-Xmin)//res*0.64).int().tolist()
        feetY=Motion[0,:,13+feet].view(-1,1);   idy=((feetY-Ymin)//res*0.64).int().tolist()
        Hterr=terr[0,2,idy,idx]+minTerr
        # print(Hterr)
        # Hterr=torch.tensor( abs(Hterr[:,0]-0.4)==0 , dtype=torch.float)*Hterr
        sign=torch.tensor(Motion[0,:,14+feet] > Hterr[:,0], dtype=torch.float)
        Motion[0,:,14+feet] = sign*Motion[0,:,14+feet] + (1-sign)*(Hterr[:,0]+0.03)
        # Revise feet force z
        sign=torch.tensor(Motion[0,:,26+feet] > 0, dtype=torch.float)
        Motion[0,:,26+feet] = sign*Motion[0,:,26+feet]



    tf=int(round(time.time() * 1000))

    TIME.append(tf-ts)
    TIME2.append(tf-ts2)
    print("Inference Time (w/ encoder) : ",tf-ts,"ms")
    print("Inference Time (w/o encoder): ",tf-ts2,"ms")
    print("Num it    : ",len(TIME))
    print("Mean Time(w\ encoder)  : ",np.mean(TIME),"ms")
    print("Mean Time(w/o encoder) : ",np.mean(TIME2),"ms")

    # Ver A. Generate ipopt variables form
    # MotionVar = GenVar(Motion[0])
    # TOWRmsg[-1].init_traj = MotionVar

    # Ver B. Publish DeepQMP directly
    publish_state_des(pub, Motion[0]);

    path_flag[-1] = False

def spinOnce(rate):
    r = rospy.Rate(rate)
    r.sleep()

if __name__=='__main__':
    # Initialize ROS python
    print("Load DeepQMP_TEST!")
    rospy.init_node('DeepQMP', anonymous = False)
    # pub_init = rospy.Publisher('/InitPath',Float64MultiArray,queue_size=1)
    pub_init = rospy.Publisher('/DeepQMPcommand',TowrCommand,queue_size=1)
    pub_traj = rospy.Publisher('/xpp/state_des',RobotStateCartesian,queue_size=1)
    pub_terr = rospy.Publisher('/opt_terrain',TowrCommand,queue_size=1)
    rospy.Subscriber("/towr/user_command", TowrCommand, TOWRCallback, queue_size=1)
    rospy.Subscriber("xpp/terrain", MarkerArray, TerrainCallback, queue_size=1)

    # Set netowrk structure
    params = torch.load("/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/model/TerrainEncoder_64x64.tar", map_location = "cpu")
    Encoder = TeerainEncoder()
    Encoder.load_state_dict(params['model_state_dict'])
    Encoder.eval()

    params = torch.load("/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/model/DeepQMP.tar", map_location = "cpu")
    # params = torch.load("/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/model/DeepQMP_Basic_step.tar", map_location = "cpu")
    Planner = DeepQMP()
    Planner.load_state_dict(params['model_state_dict'])
    Planner.eval()

    # Set initial state and goal state
    Init=[1.1778e-03, -1.0302e-03,  5.0001e-01, -4.2223e-03, -4.7900e-04,
          3.8437e-03,  5.9884e-02, -5.2376e-02,  1.3415e-03, -1.6925e-01,
         -3.2622e-02,  1.2686e-01,  3.4900e-01,  2.1350e-01,  0.0000e+00,
          3.4900e-01, -2.1350e-01,  0.0000e+00, -3.4900e-01,  2.1350e-01,
          0.0000e+00, -3.4900e-01, -2.1350e-01,  0.0000e+00,  5.5507e-02,
         -4.6469e-02,  3.9949e-01,  2.9029e-03, -2.4341e-03,  7.0018e-01,
          1.2435e-02, -1.1005e-02,  7.3960e-01,  4.6086e-02, -4.0272e-02,
          3.3032e-01,  1.0000e+00,  1.0000e+00,  1.0000e+00,  1.0000e+00]
    Init = torch.tensor(Init).view(1,-1)

    re_size = 64
    transform = transforms.Compose(
    [
     transforms.ToPILImage(),
     transforms.Resize((re_size, re_size)),
     transforms.ToTensor(),
    ])
    # if Terrain_ == 0:
    #     TESTterrain = torch.tensor(np.load('/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/data/Terrain_XYZ.npy')).reshape(1,3,100,100)
    #     # TESTterrain = torch.tensor(TESTterrain.reshape(1,3,100,100))
    # if Terrain_ == 1:
    #     TESTterrain = torch.zeros(1,1,100,100)

    # Flat
    TESTterrain = torch.zeros(1,3,100,100)

    # Basic Step
    # TESTterrain = torch.tensor(np.load('/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/data/Terrain_XYZ.npy')).reshape(1,3,100,100) # Default step terrain

    # Rand Step
    # TESTterrain = torch.tensor(np.load('/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/data/randomStep.npy')).reshape(1,3,100,100) # Random step terrain sample
    # TESTterrain = torch.flip(TESTterrain, dims=(2,))

    # Rand Gap
    # TESTterrain = torch.tensor(np.load('/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/data/randomGap.npy')).reshape(1,3,100,100) # Random gap terrain sample
    # TESTterrain[0,2] -= TESTterrain[0,2].min()


    # Nonflat
    # TESTterrain = torch.tensor(np.load('/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/data/nonflat.npy')).reshape(1,3,100,100) # Default step terrain

    # TESTterrain = terrain
    terrain_test_dataset = GenTerrainDataset(TESTterrain, transform=transform)
    terrain_test_dataloader = DataLoader(terrain_test_dataset,batch_size=1,shuffle=False)

    global TERRAIN
    _, TERRAIN = next(iter(terrain_test_dataloader))
    TERRAIN[0,0]=torch.linspace(-5,5,64).view(1,-1).repeat(64,1)
    TERRAIN[0,1]=torch.linspace(-5,5,64).view(-1,1).repeat(1,64)
    # print(TERRAIN[0,2].max())
    # print(TERRAIN[0,2].min())
    del terrain_test_dataset,terrain_test_dataloader

    # Generate initial trajectory
    while not rospy.is_shutdown():
        spinOnce(10000)
        if path_flag[-1]==True:
            print("Main TerrainCallback : ",TERRAIN[0,2].max())
            print("Main TerrainCallback : ",TERRAIN[0,2].min())
            EstimatePath(Encoder, Planner, Init, TERRAIN, pub_traj)
            # if TOWRmsg[-1].terrain != TOWRmsg[0].terrain:
            #     pub_terr.publish(TOWRmsg[-1])
            pub_terr.publish(TOWRmsg[-1])
        pub_init.publish(TOWRmsg[-1])
        # print(TOWRmsg[-1])
