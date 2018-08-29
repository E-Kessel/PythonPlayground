# -*- coding: utf-8 -*-
"""
Created on Sat Aug 18 09:20:17 2018

@author: redne
"""

import Drivey.MotorData as mdat
import Drivey.SimData as simdat
import os

path = os.path.dirname(mdat.__file__)
#print(path)

motor_CIM       = mdat.MotorData(path+'\\cim-motor-curve-data-20151104.csv')
motor_MiniCIM   = mdat.MotorData(path+'\\mini-cim-motor-curve-data-20151207.csv')
motor_BAG       = mdat.MotorData(path+'\\bag-motor-curve-data-20151207.csv')
motor_775Pro    = mdat.MotorData(path+'\\775pro-motor-curve-data-20151208.csv')