#!/usr/bin/env python
# license removed for brevity


# This file is a basic structure to write a controller that
# communicates with ROS. It will be the students responsibility
# tune the gains and fill in the missing information

# As an example this file contains PID gains, but other
# controllers use different types of gains so the class
# will need to be modified to accomodate those changes

import rospy
import time
from whirlybird_msgs.msg import Command
from whirlybird_msgs.msg import Whirlybird
from std_msgs.msg import Float32


class Controller():

    def __init__(self):

        # get parameters
        try:
            param_namespace = '/whirlybird'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Parameters not set in ~/whirlybird namespace')
            rospy.signal_shutdown('Parameters not set')

        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']
        Fe = g/l1*(m1*l1-m2*l2)
        b_psi = l1*Fe / (m1*l1**2+m2*l2**2+Jz)
        b0 = l1 / (m1*l1**2+m2*l2**2+Jy)


        # tune gains for phi
        tr_phi = 0.3
        Wn_phi = 2.2 / tr_phi
        h_phi = .7

        # tune gains for theta
        tr_theta = 1.4
        Wn_theta = 2.2 / tr_theta
        h_theta = .707

        # tune gains for psi
        tr_psi = 4.5 * tr_phi
        Wn_psi = 2.2 / tr_psi
        h_psi = .7

        # Roll Gains
        self.P_phi_ = Jx * Wn_phi**2
        self.I_phi_ = 0.0
        self.D_phi_ = Jx*(2*h_phi*Wn_phi)
        self.Int_phi = 0.0
        self.prev_phi = 0.0

        # Pitch Gains
        self.theta_r = 0.0
        self.P_theta_ = Wn_theta**2/b0
        self.I_theta_ = 0.0
        self.D_theta_ = 2.0*h_theta*Wn_theta/b0
        self.prev_theta = 0.0
        self.Int_theta = 0.0

        # Yaw Gains
        self.psi_r = 0.0
        self.P_psi_ = Wn_psi**2 / b_psi
        self.I_psi_ = 0.0
        self.D_psi_ = 2*h_psi*Wn_psi / b_psi
        self.prev_psi = 0.0
        self.Int_psi = 0.0


        self.prev_time = rospy.Time.now()

        self.Fe = Fe


        self.command_sub_ = rospy.Subscriber('whirlybird', Whirlybird, self.whirlybirdCallback, queue_size=5)
        self.psi_r_sub_ = rospy.Subscriber('psi_r', Float32, self.psiRCallback, queue_size=5)
        self.theta_r_sub_ = rospy.Subscriber('theta_r', Float32, self.thetaRCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('command', Command, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def thetaRCallback(self, msg):
        self.theta_r = msg.data


    def psiRCallback(self, msg):
        self.psi_r = msg.data


    def whirlybirdCallback(self, msg):
        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']

        phi = msg.roll
        theta = msg.pitch
        psi = msg.yaw

        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now

        ##################################
        # Implement your controller here

        # Calculate force
        F_tilde = self.P_theta_*(self.theta_r-theta)-self.D_theta_*(theta-self.prev_theta)/dt
        F = F_tilde + self.Fe

        # Calculate torque, remember that Tau_equilibrium = 0 so Tau = Tau_tilde
        phi_c = self.P_psi_*(self.psi_r-psi)-self.D_psi_*(psi-self.prev_psi)/dt
        Tau = self.P_phi_*(phi_c-phi)-self.D_phi_*(phi-self.prev_phi)/dt

        # Update states
        self.prev_theta = theta
        self.prev_psi = psi
        self.prev_phi = phi

        # Calculate Fl and Fr
        left_force  = (F + Tau/d) / 2.0
        right_force = (F - Tau/d) / 2.0
        ##################################

        # Scale Output
        l_out = left_force/km
        if(l_out < 0):
            l_out = 0
        elif(l_out > 0.7):
            l_out = 0.7 # saturate to 0.7

        r_out = right_force/km
        if(r_out < 0):
            r_out = 0
        elif(r_out > 0.7):
            r_out = 0.7 # saturate to 0.7

        # Pack up and send command
        command = Command()
        command.left_motor = l_out
        command.right_motor = r_out
        self.command_pub_.publish(command)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass
