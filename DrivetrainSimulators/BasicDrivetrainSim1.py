# -*- coding: utf-8 -*-
"""
Created on Sat Aug  4 12:33:32 2018

@author: Ethan Kessel
"""

import matplotlib.pyplot as plot
import numpy as np
import scipy.constants as const


class motor:    # NOTE: constructor takes RPM as argument for free speed, stores as rad/s
    def __init__(self, aFreeSpeed_RPM, aStallCurrent_A, aStallTorque_Nm, aSpecVolts = 12.0):
        self.speedFree_rps = aFreeSpeed_RPM * (2 * const.pi) / 60    # motor free speed, rad/s
        self.stallCurrent_A = aStallCurrent_A                       # motor stall current, amps
        self.stallTorque_Nm = aStallTorque_Nm                       # motor stall torque, N*m  
        self.specVoltage = aSpecVolts                              # motor specification voltage
        
        # -----Derived Constants
        self.ratioTI_NmpA = self.stallTorque_Nm / self.stallCurrent_A
        
    def getFreeSpeed_RPM(self):
        return self.speedFree_rps / (2 * const.pi) * 60

    # free speed (RPM), stall current (A), stall torque (N*m), (spec voltage (V))  
motors = {'CIM':     motor(5330,  131, 2.41),\
          'MiniCIM': motor(5840,  89,  1.41),\
          'BAG':     motor(13180, 53,  0.43),\
          '775Pro':  motor(18730, 134, 0.71)}

class simulationConstants:
    # NOTE: Constructor takes cm as argument for wheel radius, stores as meters
    def __init__(self, aMass_kg, aCfS, aCfK, aRatio, aRadius_cm,\
                 aMotor, aNumMotors, aGrC_Nm, aGeC, aRrC_Npmps,\
                 aBattVolts, aSimTime_s = 1.5, aDT_s = 0.001,\
                 aRbatt_O = 0.013, aRmotor_O = 0.002):
        self.mass_kg = aMass_kg     # robot mass - kg
        self.frictionStatic = aCfS  # static friction coefficient
        self.frictionDynamic = aCfK # dynamic friction coefficient
        
        self.ratio = aRatio                     # gearbox ratio
        self.wheelRadius_m = aRadius_cm / 100   # wheel radius - meters
        
        #print("Class is: ")
        #print(type(aMotor))
        if (type(aMotor) == motor):
            self.motor = aMotor
        else:
            raise ValueError('aMotor must be of class motor')
        self.numMotors = aNumMotors
        
        self.gearboxFriction_Nm = aGrC_Nm   # gearbox resistance constant
                                            # (torque required to overcome gearbox friction) - N*m
        
        self.gearboxEfficiency = aGeC               # gearbox efficiency coefficient (energy loss in gears)
        self.rollingResistance_Npmps = aRrC_Npmps   # rolling resistance constant (losses due to speed) - N/(m/sec)
        
        self.batteryVolts = aBattVolts  # actual battery starting voltage
        self.resBatt_Ohm = aRbatt_O         # battery, breaker, wiring resistance to PDP - Ohms
        self.resMotor_Ohm = aRmotor_O       # motor, breaker, wiring resistance to PDP - Ohms
        
        self.maxSimTime_s = aSimTime_s
        self.simTimeStep_s = aDT_s
        
        # -----Derived Constants-----
        self.torqueOffset_Nm = (self.motor.stallTorque_Nm * self.batteryVolts)/\
                               (self.motor.specVoltage + self.motor.stallCurrent_A * self.resMotor_Ohm +\
                                self.motor.stallCurrent_A * self.numMotors * self.resBatt_Ohm)
                               
#        self.torqueSlope_Nmprps = (self.motor.stallTorque_Nm * self.batteryVolts)/\
#                                  (self.motor.speedFree_rps * (self.motor.specVoltage +\
#                                                               self.motor.stallCurrent_A * self.resMotor_O +\
#                                                               self.motor.stallCurrent_A * self.numMotors * self.resBatt_O))
        self.torqueSlope_Nmprps = self.torqueOffset_Nm / self.motor.speedFree_rps
        
        self.NmPerAmp = self.motor.stallTorque_Nm / self.motor.stallCurrent_A
                                  
        self.weight_N = self.mass_kg * const.g
        
        self.ampsPerNewton = self.wheelRadius_m / (self.numMotors * self.gearboxEfficiency * self.ratio * self.NmPerAmp)
        
class simulationData:
    def __init__(self, aConstants): 
        self.simConsts = aConstants
        
        self.timeStamps_s = np.arange(0.0, self.simConsts.maxSimTime_s, self.simConsts.simTimeStep_s)
        
        self.slip = np.zeros(len(self.timeStamps_s))
        self.motorCurrent = np.zeros(len(self.timeStamps_s))
        self.motorVoltage = np.zeros(len(self.timeStamps_s))
        self.d_m = np.zeros(len(self.timeStamps_s))
        self.v_mps = np.zeros(len(self.timeStamps_s))
        self.a_mps2 = np.zeros(len(self.timeStamps_s))
        
        self.a_mps2[0] = self.accel(0, 0)
    
    def accel(self, iVelo, i):
        motorSpeed_rps = iVelo / self.simConsts.wheelRadius_m * self.simConsts.ratio
        motorTorque_Nm = self.simConsts.torqueOffset_Nm - self.simConsts.torqueSlope_Nmprps * motorSpeed_rps
        wheelTorque_Nm = motorTorque_Nm * self.simConsts.ratio * self.simConsts.gearboxEfficiency
        wheelForce_N = wheelTorque_Nm / self.simConsts.wheelRadius_m * self.simConsts.numMotors
        
        if (wheelForce_N > self.simConsts.weight_N * self.simConsts.frictionStatic):
            self.slip[i] = True
        elif (wheelForce_N < self.simConsts.weight_N * self.simConsts.frictionDynamic):
            self.slip[i] = False
            
        if (self.slip[i] == True):
            effectiveForce_N = self.simConsts.weight_N * self.simConsts.frictionDynamic
        else:
            effectiveForce_N = wheelForce_N
            
        self.motorCurrent[i] = effectiveForce_N * self.simConsts.ampsPerNewton
        
        self.motorVoltage[i] = self.simConsts.batteryVolts -\
            self.motorCurrent[i]*(self.simConsts.numMotors * self.simConsts.resBatt_Ohm - self.simConsts.resMotor_Ohm)
        if (effectiveForce_N > self.simConsts.gearboxFriction_Nm):   # units here are confusing, fix this concept later
            effectiveForce_N -= self.simConsts.gearboxFriction_Nm
        else:
            effectiveForce_N = 0.0
            
        return effectiveForce_N / self.simConsts.mass_kg
    
    def heun(self):
        i = 1
        for t in self.timeStamps_s[1:]:
        #while (self.a_mps2[i-1] > 0.01):
            nowVel = self.v_mps[i-1] + self.a_mps2[i-1] * self.simConsts.simTimeStep_s
            nowAcc = self.accel(nowVel, i)
            nowVel = self.v_mps[i-1] + (self.a_mps2[i-1] + nowAcc)/2 * self.simConsts.simTimeStep_s
            self.a_mps2[i] = self.accel(nowVel, i)
            self.v_mps[i] = nowVel
            self.d_m[i] = self.d_m[i-1] + (self.v_mps[i-1] + nowVel)/2 * self.simConsts.simTimeStep_s
            
            i += 1
    
if __name__ == '__main__':
    my_Consts = simulationConstants(60.0,               # robot mass (kg)
                                    1.0,                # static friction coefficient
                                    0.8,                # dynamic friction coefficient
                                    10.00,               # gearbox ratio
                                    7.62,               # wheel radius (cm)
                                    motors['CIM'],      # gearbox motor
                                    4,                  # number of motors
                                    10.0,               # gearbox resistance constant (N*m)?
                                    0.9,                # gearbox efficiency coefficient
                                    0.001,              # rolling resistance constant (N/(m/s))
                                    12.7)               # battery starting volatge (V)
                                    #1.5,                # maximum simulation time (s)
                                    #0.001)              # simulation timestep (s)
                                                        # battery circut resistance (ohms)
                                                        # motor circut resistance (ohms)                                                        
    simData = simulationData(my_Consts)
    
    simData.heun()
    
    plot.figure(1)
    plot.cla()
    plot.grid()
    plot.plot(simData.timeStamps_s, simData.d_m,
              simData.timeStamps_s, simData.v_mps,
              simData.timeStamps_s, simData.a_mps2,
              simData.timeStamps_s, simData.slip,
              simData.timeStamps_s, simData.motorVoltage,
              simData.timeStamps_s, simData.motorCurrent/10)
    plot.legend(['D','V','A','Slip','Volts','Amps/10'])
    