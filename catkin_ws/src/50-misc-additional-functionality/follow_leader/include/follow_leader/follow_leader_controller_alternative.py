import math
import numpy as np
import rospy
import control
import duckietown_utils as dtu

class FollowLeaderController():

    def __init__(self):

        #Constants
        self.v_ref = 0.22
        self.d_int = 0.
        self.q = np.array([[70., 0], [0, 3.]])
        self.r = np.array([0.007])

        self.rho_ref = 0.16
        self.k_rho_p = -0.9
        self.k_rho_i = 0
        self.k_rho_d = -0.2

        self.err_rho_int = 0
        self.err_rho_last = 0


    def getControlOutput(self, fl, rho, theta, psi, dt_last):
        ################## Begin LQR control for omega####################
        # Compute d and phi
        d_est = math.sin(theta + psi) * rho
        phi_est = -psi
        #Exception for dt_last=0
        #Exception handler for dt_last==0
        if (dt_last==0) : dt_last=0.0001
        x = np.array([[d_est],[phi_est]])
        #Discrete State Space Matrices
        a = np.array([[1., self.v_ref*dt_last], [0, 1.]])
        b = np.array([[self.v_ref*dt_last*dt_last/2], [dt_last]])
        #Solve Discrete Algebraic Riccati Equation
        (x_ric, l, g) = control.dare(a, b, self.q, self.r)

        omega_out = -np.dot(g, x)*1.75


        ################## End LQR control for omega####################

        ################### Start PID for v ############################
        err_rho = self.rho_ref - rho
        self.err_rho_int += err_rho*dt_last

        rho_p = self.k_rho_p * err_rho

        rho_i = self.k_rho_i * self.err_rho_int

        rho_d = self.k_rho_d * (err_rho - self.err_rho_last) / dt_last

        v_out = rho_p + rho_i + rho_d

        if v_out < 0.05:
            v_out = 0
            omega_out = 0
        ################### End PID for v ############################

        # return commands
        self.err_rho_last = err_rho
        return (v_out, omega_out)

    def updateParams(self, v_ref, d_int, q, r):
        self.v_ref = v_ref
        self.d_int = d_int
        self.q = q
        self.r = r
