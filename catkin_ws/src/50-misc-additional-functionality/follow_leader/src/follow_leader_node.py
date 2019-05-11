#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, LanePose, FSMState, BoolStamped, VehiclePose
from duckietown_msgs.srv import SetFSMState
from follow_leader.follow_leader_controller import FollowLeaderController
import os, imp, time
import sys

class FollowLeaderNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

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

    def stopVeh(self):
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)

    def cbDetected(self, veh_detected):
        self.detected = veh_detected.data
        rospy.loginfo("[%s] detected: [%u] " %(self.node_name,self.detected))
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
            v_out, omega_out = self.controller.getControlOutput(rho, theta, psi, dt_last)

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


if __name__ == "__main__":
    rospy.init_node("follow_leader_node",anonymous=False)
    follow_leader_node = FollowLeaderNode()
    rospy.spin()
