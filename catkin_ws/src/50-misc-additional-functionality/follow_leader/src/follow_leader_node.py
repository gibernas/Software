#!/usr/bin/env python
import rospy
import math
import numpy as np
from duckietown_msgs.msg import Twist2DStamped, LanePose, FSMState, BoolStamped, VehiclePose
from duckietown_msgs.srv import SetFSMState
from follow_leader.follow_leader_controller_alternative import FollowLeaderController
import os, imp, time
import sys

class FollowLeaderNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Setup parameters
        self.setupParams()

        # Publicaitons
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions

        # Setup subscriptions for follower
        self.sub_veh_pos = rospy.Subscriber("~veh_pos", VehiclePose, self.cbFollowLeader, queue_size=1)
        self.sub_veh_detected = rospy.Subscriber("~vehicle_detected", BoolStamped, self.cbDetected, queue_size=1)


        # Services
        self.srv_fsm_mode = rospy.ServiceProxy('~fsm_mode', SetFSMState)
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)


        # HACK: Add listener for FSM machine in order to avoid integrating if not in autopilot mode
        veh_name = os.environ['VEHICLE_NAME']
        self.sub_fsm_mode = rospy.Subscriber("/" + str(veh_name) + "/fsm_node/mode", FSMState, self.cbMode, queue_size=1)

        # Set up operating variable, only if in right state
        self.active = False

        self.controller = FollowLeaderController()

        self.last_ms = None
        self.detected = False

        ## Parameters update timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(2.0), self.updateParams)

    def stopVeh(self):
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)

    def cbDetected(self, veh_detected):
        self.detected = veh_detected.data
        if not self.detected:
            self.stopVeh()

    def cbFollowLeader(self, pose_msg):
        if not self.detected:
            self.stopVeh()
        else:
            # Calculate time since last command
            currentMillis = int(round(time.time() * 1000))
            if self.last_ms is not None:
                dt_last = (currentMillis - self.last_ms) / 1000.0
            else:
                dt_last = 0

            # Obtain parameters for controller
            rho = pose_msg.rho.data
            theta = pose_msg.theta.data
            psi = pose_msg.psi.data

            # Obtain new v and omega from controller
            v_out, omega_out = self.controller.getControlOutput(self, rho, theta, psi, dt_last)

            # Update last timestamp
            self.last_ms = currentMillis

            # Create message and publish
            car_control_msg = Twist2DStamped()
            car_control_msg.header = pose_msg.header
            car_control_msg.v = v_out
            car_control_msg.omega = omega_out
            self.publishCmd(car_control_msg)

    # FSM
    def cbMode(self,fsm_state_msg):
        self.operating = fsm_state_msg.state == "LANE_FOLLOWING"

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)

    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)


    # Start of parameter server
    def setupParams(self):
        self.q = self.setupParam("~q", [[80., 0, 0], [0, 40., 0], [0, 0, 2]])
        self.r = self.setupParam("~r", [0.007])
        self.v_ref = self.setupParam("~v_ref", 0.25)
        self.d_int = self.setupParam("~d_int", 0.)

    def updateParams(self,event):
        self.q = rospy.get_param("~q")
        self.r = rospy.get_param("~r")
        self.v_ref = rospy.get_param("~v_ref")
        self.d_int = rospy.get_param("~d_int")

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value



if __name__ == "__main__":
    rospy.init_node("follow_leader_node",anonymous=False)
    follow_leader_node = FollowLeaderNode()
    rospy.spin()
