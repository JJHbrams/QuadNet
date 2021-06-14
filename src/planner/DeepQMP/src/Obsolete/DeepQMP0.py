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

import time

class GenTerrainDataset(Dataset):

  def __init__(self, TerrainData, transform=None):
      self.data = TerrainData.float()
      self.transform = transform

  def __getitem__(self, index):
      data = self.data
      if self.transform:
            data = self.transform(data[index])

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

MaxSampleNum = 201
Path_dim = 40
z_dim = 128
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
        hidden_dim = [128,40]
        output_dim=40
        goal_dim=6

        alpha=0.01

        # CNN
        ch1 = 24
        self.ConvReal = nn.Sequential(
                                              nn.Conv2d(1, ch1, (3, 1), stride=(3,1)),
                                              nn.BatchNorm2d(ch1),
                                              # nn.LeakyReLU(alpha, inplace=True),
                                              mish(),
                                  )
        self.ConvBin = nn.Sequential(
                                              nn.Conv2d(1, ch1, (4, 1), stride=1),
                                              nn.BatchNorm2d(ch1),
                                              mish(),
                                  )
        self.ConvEnv = nn.Sequential(
                                              nn.Conv2d(1, ch1, (1, 1), stride=1),
                                              nn.BatchNorm2d(ch1),
                                              mish(),
                                    )
        ch2 = 1
        ch3 = 8
        ch4 = 16
        ch5 = 32
        featW = 41
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
                                              mish(),
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
        self.output_dim = output_dim
        self.GenPath = nn.Sequential(
                                                nn.Linear(dslayer3, MaxSampleNum*output_dim),
                                      )

    def forward(self, X, Terrain, pathLen=MaxSampleNum-1):
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
        FullPath = self.GenPath(MLPout).view(-1,MaxSampleNum,self.output_dim)
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
    print(np.shape(BaseMotion_lin))

    baseang = torch.cat([Motions[:,3:6], Motions[:,9:12]], dim=1)
    BaseMotion_ang = baseang
    BaseMotion_ang = BaseMotion_ang[0::NUMbase,:]
    # BaseMotion_ang = torch.cat([BaseMotion_ang, baseang[-1,:].view(1,6)],dim=0)
    BaseMotion_ang = BaseMotion_ang.flatten().tolist()
    Variables += BaseMotion_ang
    print(np.shape(BaseMotion_ang))

    FEET = [0,3,6,9]
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
        print(np.shape(eeMotion))
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
        print(np.shape(eeForce))
        Variables += eeForce

    return Variables

TOWRmsg = [TowrCommand()]
Goal=torch.zeros(1,6)
Terrain_ = 0
path_flag = []
path_flag.append(False)
def TOWRCallback(msg):
    TOWRmsg.append(msg)
    Goal[0,0] = msg.goal_lin.pos.x
    Goal[0,1] = msg.goal_lin.pos.y
    Goal[0,2] = msg.goal_lin.pos.z + 0.5
    Goal[0,3] = msg.goal_ang.pos.x
    Goal[0,4] = msg.goal_ang.pos.y
    Goal[0,5] = msg.goal_ang.pos.z
    Terrain_ = msg.terrain
    path_flag.append(msg.optimize or msg.play_initialization)
    print('Goal pose : ',Goal)

def EstimatePath(Encoder, Planner, XnG):
    print('Generating Path...')
    Env = Encoder(terrain)[-1]
    Motion = Planner(XnG,Env)
    Motion[0,:,-16:-4] = (Motion[0,:,-16:-4])*100
    # Generate ipopt variables form
    MotionVar = GenVar(Motion[0])
    path_flag[-1] = False
    TOWRmsg[-1].init_traj = MotionVar

def spinOnce(rate):
    r = rospy.Rate(rate)
    r.sleep()

if __name__=='__main__':
    # Initialize ROS python
    print("Load DeepQMP_TEST!")
    rospy.init_node('DeepQMP', anonymous = False)
    # pub_init = rospy.Publisher('/InitPath',Float64MultiArray,queue_size=1)
    pub_init = rospy.Publisher('/DeepQMPcommand',TowrCommand,queue_size=1)
    rospy.Subscriber("/towr/user_command", TowrCommand, TOWRCallback, queue_size=1)

    # cuda = True if torch.cuda.is_available() else False
    cuda = False
    Tensor = torch.cuda.FloatTensor if cuda else torch.FloatTensor

    # Set netowrk structure
    params = torch.load("/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/model/TerrainEncoder.tar", map_location = "cpu")
    Encoder = TeerainEncoder()
    Encoder.load_state_dict(params['model_state_dict'])
    Encoder.eval()

    params = torch.load("/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/model/DeepQMP.tar", map_location = "cpu")
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
    if Terrain_ == 0:
        TESTterrain = torch.tensor(np.load('/home/mrjohd/Quadrupeds_ws/src/planner/DeepQMP/data/Terrain_XYZ.npy'))
        TESTterrain = torch.tensor(TESTterrain[2].reshape(1,1,100,100))
    if Terrain_ == 1:
        TESTterrain = torch.zeros(1,1,50,50)
    terrain_test_dataset = GenTerrainDataset(TESTterrain, transform=transform)
    terrain_test_dataloader = DataLoader(terrain_test_dataset,batch_size=1,shuffle=False)
    _, img = next(iter(terrain_test_dataloader))
    terrain = img/img.max()

    # Generate initial trajectory
    while not rospy.is_shutdown():
        spinOnce(10000)
        if path_flag[-1]==True:
            XnG = torch.cat([Init, Goal],dim=1).type(Tensor)
            EstimatePath(Encoder, Planner, XnG)
        pub_init.publish(TOWRmsg[-1])
        # print(TOWRmsg[-1])
