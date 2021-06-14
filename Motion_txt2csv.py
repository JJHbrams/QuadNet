#!/usr/bin/env python3
# python
import numpy as np
import math
import pandas as pd
from random import randint
import copy as cp

import torch

TerrainData = pd.read_csv('./src/data/csv/01.Motion_data.txt')
TerrainData = TerrainData[5:]

'''
  Feet velocity
'''
# VLF
data=TerrainData.values
domain=14
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_LFx',footVel)

data=TerrainData.values
domain=15
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_LFy',footVel)

data=TerrainData.values
domain=16
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_LFz',footVel)

# VRF
data=TerrainData.values
domain=20
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_RFx',footVel)
data=TerrainData.values
domain=21
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_RFy',footVel)
data=TerrainData.values
domain=22
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_RFz',footVel)

# VLH
data=TerrainData.values
domain=26
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_LHx',footVel)
data=TerrainData.values
domain=27
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_LHy',footVel)
data=TerrainData.values
domain=28
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_LHz',footVel)

# VRH
data=TerrainData.values
domain=32
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_RHx',footVel)
data=TerrainData.values
domain=33
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_RHy',footVel)
data=TerrainData.values
domain=34
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'V_RHz',footVel)

'''
  Acceleration
'''

# AccLF
data=TerrainData.values
domain=17
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_LFx',footVel)
data=TerrainData.values
domain=18
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_LFy',footVel)
data=TerrainData.values
domain=19
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_LFz',footVel)

# AccRF
data=TerrainData.values
domain=26
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_RFx',footVel)
data=TerrainData.values
domain=27
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_RFy',footVel)
data=TerrainData.values
domain=28
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_RFz',footVel)

# AccLH
data=TerrainData.values
domain=35
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_LHx',footVel)
data=TerrainData.values
domain=36
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_LHy',footVel)
data=TerrainData.values
domain=37
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_LHz',footVel)

# AccRH
data=TerrainData.values
domain=44
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_RHx',footVel)
data=TerrainData.values
domain=45
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_RHy',footVel)
data=TerrainData.values
domain=46
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+3,'a_RHz',footVel)

# BaseA
data=TerrainData.values
domain=8
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+6,'aCOMx',footVel)
domain=9
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+6,'aCOMy',footVel)
domain=10
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+6,'aCOMz',footVel)
domain=11
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+6,'dWdx',footVel)
domain=12
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+6,'dWdy',footVel)
domain=13
footVel = (data[1:,domain]-data[:-1,domain])/0.01
footVel=np.insert(footVel, 0, 0., axis=0)
TerrainData.insert(domain+6,'dWdz',footVel)


TerrainData.to_csv('~/GN_NMPC_TO_Distribution/build/bin/myTest.csv',header=False)
TerrainData
