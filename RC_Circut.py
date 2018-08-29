# -*- coding: utf-8 -*-
"""
Created on Sun Aug 19 09:43:21 2018

@author: redne
"""

from scipy.integrate import ode
import numpy as np
import matplotlib.pyplot as pl

def f(t, V, R, C):
    return -V/(R*C)

def g(t, V, R, C, Vin):
    return (V-Vin)/(R*C)

t0 = 0.0
tstop = 2.0
dt = 0.001

startVolts = 5.0
Cap_f = 0.001
res_ohm = 250

ts = np.arange(t0, tstop, dt)
volts = np.zeros(len(ts))
volts[0] = startVolts

myInt = ode(f).set_integrator('dopri5')
myInt.set_initial_value(startVolts, t0).set_f_params(res_ohm, Cap_f)

i = 1
while myInt.successful() and myInt.t < tstop and myInt.y > 0 and i < len(ts):
    myInt.integrate(myInt.t + dt)
    volts[i] = myInt.y
    i = i + 1
    
pl.figure(1)
pl.cla()
pl.grid()
pl.plot(ts,volts)
pl.xlabel('Time(s)')
pl.ylabel('Volts')