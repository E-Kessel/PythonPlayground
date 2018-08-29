# -*- coding: utf-8 -*-
"""
Created on Sat Aug 18 09:54:30 2018

@author: redne
"""

import Drivey.MotorData as mdat
import Drivey.Circut as circut

import numpy as np
import scipy.constants as const

class simConstants:
    def __init__(self, aM_kg, aCfS, aCfK, aR_cm, aRatio, aMotorDat, aNumM, aGeC, aGrC_Nm, aGrr_Nmpmps, aCircut,
                 aSimT_s = 1.5, aDt_s = 0.001):
        self.mass_kg = aM_kg
        self.kFricStatic = aCfS
        self.kFricKinetic = aCfK
        self.radius_m = aR_cm / 100
        self.gearRatio = aRatio
        
        if (type(aMotorDat) == mdat.MotorData):
            self.motor = aMotorDat
        else:
            raise ValueError('aMotorDat must be of class MotorData')
        self.numM = aNumM
        
        self.kGearEfficiency = aGeC
        self.gearboxResistance_Nm = aGrC_Nm
        self.rollingRes_Nmpmps = aGrr_Nmpmps
        
        if (type(aCircut) == circut.Circut):
            self.circut = aCircut
        else:
            raise ValueError('aCircut must be of class Circut')
            
        self.simTime_s = aSimT_s
        self.dt_s = aDt_s
        
        #----------------------------------------------------------------------
        
        self.w_N = self.mass_kg * const.g        
        
    def getTorqueOffset(self, aVoltage):
        with self.motor as m:
            with self.circut as c:
                return (m.stallT_Nm * aVoltage) / (m.specVolts + m.stallI_A * (c.resMotor_Ohm + self.numM * c.resBatt_Ohm))
            
    def getTorqueSlope(self, aVoltage):
        return self.getTorqueOffset(aVoltage) / self.motor.freeSpeed_radps
    
    def getTorqueAtSpeed(self, aVolts, aSpeed):
        return self.getTorqueOffset(aVolts) - self.getTorqueSlope(aVolts) * aSpeed
    
class simData:
    def __init__(self, aSimConsts):
        self.simConsts = aSimConsts
        
        self.timeStamps_s = np.arange(0.0, self.simConsts.simTime_s, self.dt_s)
        
        size = len(self.timeStamps_s)
        self.slip = np.zeros(size)
        self.mI_A = np.zeros(size)
        self.mVolts = np.zeros(size)
        self.mSpd_radps = np.zeros(size)
        self.d_m = np.zeros(size)
        self.v_mps = np.zeros(size)
        self.a_mps2 = np.zeros(size)