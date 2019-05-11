import math
import numpy as np
import rospy

class FollowLeaderController():

    def __init__(self):
        self.v_ref = 0.22
        self.rho_ref = 0.06
        self.theta_ref = 0

        #PID gains
        self.k_P_rho = 4
        self.k_I_rho = 0
        self.k_D_rho = 1

        self.k_P_theta = 5
        self.k_P_theta_big = 4
        self.k_I_theta = 0
        self.k_D_theta = 1

        #initialisation variables (not to be changed!!!)
        self.C_I_rho = 0
        self.C_D_rho = 0
        self.err_old_rho = 0
        self.C_I_theta = 0
        self.err_old_theta = 0

    def getControlOutput(self, rho, theta, psi, dt_last):

        #PID for rho
        #calculate error
        err_rho = rho - self.rho_ref

        #calculate p gain
        C_P_rho = self.k_P_rho * err_rho
        #calculate Integral gain
        self.C_I_rho = self.C_I_rho + dt_last  * err_rho
        C_I_rho = self.C_I_rho* self.k_I_rho
        #calculate derivative gain
        if dt_last > 0:
            self.C_D_rho = self.k_D_rho * (err_rho - self.err_old_rho)/dt_last
        else:
            self.C_D_rho = 0.01
        self.err_old_rho = err_rho
        #compute output of v (velocity) problem mit C-D-rho ??
        v_out = self.v_ref * (C_P_rho + C_I_rho + self.C_D_rho)
        #prevents duckiebot of driving backwards
        if v_out < 0:
            v_out = 0
        #end of PID for rho #################################################################################################################################################

        #PID for theta ###########################################################################################################################################
        #calculate error
        err_theta = self.theta_ref - theta
        #calculate p gain, if psi is big enough increase gain
        if abs(psi) > 0.6:
            C_P_theta = self.k_P_theta_big * err_theta
        else:
            C_P_theta = self.k_P_theta * err_theta
        #calculate Integral gain
        self.C_I_theta = self.C_I_theta + dt_last  * err_theta
        C_I_theta = self.C_I_theta* self.k_I_theta
        # calculate derivative gain
        if dt_last > 0:
            C_D_theta = self.k_D_theta * (err_theta - self.err_old_theta)/dt_last
        else:
            C_D_theta = 0
        self.err_old_theta = err_theta
        #compute output of v (velocity)
        omega_out = C_P_theta + C_I_theta + C_D_theta
        #end of PID for theta #################################################################################################################################################
        return (v_out, omega_out)
