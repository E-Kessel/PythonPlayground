# -*- coding: utf-8 -*-
"""
Created on Fri Aug 17 20:13:03 2018

@author: redne
"""

import csv
import numpy as np
import scipy.constants as const
#from recordtype import recordtype as rt

class MotorDataError(Exception):
    pass 

class MotorData:
    #NOTE: when importing the data from the CSV file, the folowing criteria are expected
    #   Columns ordered: speed(RPM), torque(Nm), current(A), supplied power(W), output pwr(W), efficiency(%), pwr dissipation(W)
    #   First row must be header
    #   Sorted from lowest speed to highest
    #Speed is converted from Revolutions per Minute into Radians per Second (more useful in pys calcs)
    #Efficiency is converted to a decimal 
    class MotorCurves:
        def __init__(self, size, has_header = False):
#            # Create recordtype MotorCurves to store motor curve data
#            # This functions like a namedtuple, but is mutable
#            MotorCurves = rt('Motor Data Curves',\
#                                  ['speed_radps',
#                                   'torque_Nm',
#                                   'current_A',
#                                   'pSupplied_W',
#                                   'pOutput_W',
#                                   'efficiency',
#                                   'pDissipation_W'])
#            curveData = MotorCurves()
            #print("Init Motor")      
            self.speed_radps = np.zeros(size)
            self.torque_Nm = np.zeros(size)
            self.current_A = np.zeros(size)
            self.pSupplied_W = np.zeros(size)
            self.pOutput_W = np.zeros(size)
            self.efficiency = np.zeros(size)
            self.pDissipation_W = np.zeros(size)
            
        def fill(self, motorData_CSV, startLine = 0):
            with open(motorData_CSV, 'r') as datafile:
                print('Reading %s' % motorData_CSV)
                readCSV = csv.reader(datafile)
                i = 0
                for row in readCSV:
                    if (i >= startLine):
                        # I don't like this way of typing this out multiple times
                        # invesigate recordtype tried above
                        #print(float(row[0]))
                        j = i - startLine
                        self.speed_radps[j] = float(row[0]) * (2*const.pi) / 60.0
                        self.torque_Nm[j] = float(row[1])
                        self.current_A[j] = float(row[2])
                        self.pSupplied_W[j] = float(row[3])
                        self.pOutput_W[j] = float(row[4])
                        self.efficiency[j] = float(row[5]) / 100.0
                        self.pDissipation_W[j] = float(row[6])
                    
                    i += 1
                datafile.close()    # Clean up when done
                
    def __init__(self, motorData_CSV, aSpecVolts = 12.0):
        with open(motorData_CSV, 'r') as datafile:
            readCSV = csv.reader(datafile)
            sniffCSV = csv.Sniffer()   # Use CSV sniffer to detect if file has header
            has_header = sniffCSV.has_header(datafile.read(2048))
            datafile.seek(0)
            
            # Read data from CSV into data object, ignoring first line if it is a header
            if (has_header == True):
                for n in readCSV:   #File must be read to the end to find length
                    unused = n
                self.motorCurves = self.MotorCurves(readCSV.line_num - 1)
                self.motorCurves.fill(motorData_CSV, 1)
            else:
                raise MotorDataError('Motor Data File is missing header!')
            datafile.close()    # Clean up when done
        
        #Critical Motor Data
        self.freeSpeed_radps = self.motorCurves.speed_radps[len(self.motorCurves.speed_radps)-1] # Last
        self.stallT_Nm = self.motorCurves.torque_Nm[0]
        self.stallI_A = self.motorCurves.current_A[0]
        self.specVolts = aSpecVolts
        
        self.ratioTI_NmpA = self.stallT_Nm / self.stallI_A
    
    # Function to get free speed in RPM instead of radians/sec
    def getFreeSpeed_RPM(self):
        return self.freeSpeed_radps / (2*const.pi) * 60

    