import math
import numpy as np
import rospy

class FollowLeaderController():

    def __init__(self):
        self.v_ref = 0.22

        self.k_d = -3.5
        self.k_phi = -2


    def getControlOutput(self, rho, theta, psi, dt_last):

        # Compute d and phi
        d = math.sin(theta + psi) * rho
        phi = psi

        d_err = -d
        phi_err = -phi

        v_out = self.v_ref
        omega_out = self.k_d * (0.22/v_out) * d_err + self.k_phi * (0.22/v_out) * phi_err


        return (v_out, omega_out)
