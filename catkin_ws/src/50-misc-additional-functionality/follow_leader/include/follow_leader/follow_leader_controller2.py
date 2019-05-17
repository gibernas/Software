import math
import numpy as np
import rospy
import control
import duckietown_utils as dtu

class FollowLeaderController():

    def __init__(self):

        #Constants
        self.v_ref = 0.1234
        self.d_int = 0.
        self.q = np.array([[80., 0, 0], [0, 40., 0], [0, 0, 2]])
        self.r = np.array([0.007])

        self.rho_ref = 0.11
        self.k_P1 = -1.2


    def getControlOutput(self, fl, rho, theta, psi, dt_last):
        ################## Begin LQR control for omega####################
        #Exception for dt_last=0
        if (dt_last==0) : dt_last=0.0001

        # Compute d and phi
        d_est = math.sin(theta + psi) * rho
        phi_est = -psi

        #Update state
        x = np.array([[d_est],[phi_est],[self.d_int]])
        #Adapt State Space to current time step
        a = np.array([[1., self.v_ref*dt_last, 0], [0, 1., 0], [-dt_last, -0.5*self.v_ref*dt_last*dt_last, 1]])
        b = np.array([[0.5*self.v_ref*dt_last*dt_last], [dt_last], [-self.v_ref*dt_last*dt_last*dt_last/6]])
        #Solve ricatti equation
        (x_ric, l, g) = control.dare(a, b, self.q, self.r)
        #feed trough velocity
        v_out = self.v_ref
        #Change sign on on K_I (because Ricatti equation returns [K_d, K_phi, -K_I])
        g[[0],[2]] = -g[[0],[2]]
        #Calculate new input
        omega_out = -np.dot(g, x)
        #Update integral term
        self.d_int = self.d_int + d_est * dt_last
        ################## End LQR control for omega####################

        ################### Start PID for v ############################
        err_rho = self.rho_ref - rho

        v_out = self.k_P1 * err_rho

        ################### End PID for v ############################


        # return commands
        return (v_out, omega_out)

    def updateParams(self, v_ref, d_int, q, r):
        self.v_ref = v_ref
        self.d_int = d_int
        self.q = q
        self.r = r
