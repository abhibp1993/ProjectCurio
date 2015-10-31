# -*- coding: utf-8 -*-
"""
Created on Sat Oct 10 20:22:02 2015

Represents the classes required for kinematic model of Curio.
    - Motor
    - Encoder
    - Driver: Includes the whole system from PWM quantizer to output of motor driver.
    - Curio
@author: abhibp1993
"""

import sys
import os
dirIFramework = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(dirIFramework)

import lib.sm as sm

class Motor(sm.SM):
    L  = 0.010   # armature inductance
    R  = 10      # armature resistance   
    J  = 0.1     # load
    Kb = 0.1     # back-emf constant
    Kt = 0.1     # torque-current constant
    B  = 0.01    # damping

    
    def __init__(self, currSpeed=0.0):
        self.startState = currSpeed       # current speed
        
        smElectric   = sm.Cascade(sm.Gain(1/(self.L + self.R)), sm.FeedbackSubtract(sm.Wire(), sm.Cascade(sm.R, sm.Gain(self.L/(self.L+self.R)))))
        
        smMechanical = sm.Cascade(sm.Gain(1/(self.J + self.B)), sm.FeedbackSubtract(sm.Wire(), sm.Cascade(sm.B, sm.Gain(self.J/(self.J+self.B)))))
    
        self.machine = sm.FeedbackSubtract(sm.Cascade(sm.Cascade(smElectric, sm.Gain(self.Kt)), smMechanical), sm.Gain(self.Kb))
        
        
    def getNextValues(self, state, inp):
        """
        @inp: voltage input
        @state: speed
        @newState: new speed after applying voltage for one time step interval
        @output: new speed
        """
        return self.machine.getNextValues()


