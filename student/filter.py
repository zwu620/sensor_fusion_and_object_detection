# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = params.dim_state #general parameter - process model dimension
        self.dt = params.dt # Kalman filter parameters - time increment- fixed
        self.q = params.q # Kalman filter parameters - process noise variable for Kalman filter Q
        #pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        F_matrix = np.eye((self.dim_state))
        F_matrix = np.matrix(F_matrix)
        F_matrix[0,3]=self.dt
        F_matrix[1,4]=self.dt
        F_matrix[2,5]=self.dt
        ############

        return F_matrix
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        Q_matrix = np.zeros((self.dim_state, self.dim_state))
        q_a = (self.dt ** 4 / 4)*self.q
        q_b = (self.dt ** 3 / 2)*self.q
        q_c = (self.dt ** 2) * self.q
        Q_matrix = ([[q_a, 0, 0, q_b, 0, 0],
                     [0, q_a, 0, 0, q_b, 0],
                     [0, 0, q_a, 0, 0, q_b],
                     [q_b, 0, 0, q_c, 0, 0],
                     [0, q_b, 0, 0, q_c, 0],
                     [0, 0, q_b, 0, 0, q_c]])
        
        
                
        ############

        return np.matrix(Q_matrix)
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        F = self.F()
        F_t = F.transpose()
        Q = self.Q()
        print(F)
        print(track.P)
        print(F_t)
        print(Q)
        x = F * track.x 
        P = F * track.P * F_t + Q
        
        track.set_x(x)
        track.set_P(P)

        #pass
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H)
        #I = np.asmatrix(np.eye(self.dim_state))
        K = track.P * H.transpose() * S.I
        x = track.x + (K * gamma)
        P = (np.eye(self.dim_state) - K * H) * track.P
        
        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        return meas.z - meas.sensor.get_hx(track.x) #gamma = z-Hx
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return H * track.P * H.transpose() + meas.R
        
        ############
        # END student code
        ############ 