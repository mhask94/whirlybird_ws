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
import numpy as np
import control as ctrl

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
        # rospy.loginfo('d = ',d)

        # state-space equilibrium values
        phi_e = 0.0
        theta_e = 0.0
        psi_e = 0.0

        # tune gains for phi
        tr_phi = 0.3
        Wn_phi = 2.2 / tr_phi
        h_phi = .7

        # tune gains for theta
        tr_theta = 1.4
        Wn_theta = 2.2 / tr_theta
        h_theta = .707

        # tune gains for psi
        tr_psi = 10.0 * tr_phi
        Wn_psi = 2.2 / tr_psi
        h_psi = .7

        # Check for controllability
        C1 = l1*Fe / (m1*l1**2+m2*l2**2+Jz)
        C2 = l1 / (m1*l1**2+m2*l2**2+Jy)

        A_lat = np.mat([[0,0,1,0],[0,0,0,1],[0,0,0,0],[C1,0,0,0]])
        B_lat = np.mat([[0],[0],[1.0/Jx],[0]])
        C_lat = np.mat([[0,1,0,0]])

        C_ab_lat = ctrl.ctrb(A_lat,B_lat)
        rank_lat = np.linalg.matrix_rank(C_ab_lat)
        rospy.logwarn('lat rank = %d',rank_lat)
        if rank_lat != 4:
            rospy.logwarn('Warning: Not Controllable')

        A_lon = np.mat([[0,1],[0,0]])
        B_lon = np.mat([[0],[C2]])
        C_lon = np.mat([[1.,0]])

        C_ab_lon = ctrl.ctrb(A_lon,B_lon)
        rank_lon = np.linalg.matrix_rank(C_ab_lon)
        rospy.logwarn('lon rank = %d',rank_lon)
        if rank_lon != 2:
            rospy.logwarn('Warning: Not Controllable')

        lat_poly = np.convolve([1,2*Wn_phi*h_phi,Wn_phi**2],[1,2*Wn_psi*h_psi,Wn_psi**2])
        p_lat = np.roots(lat_poly)
        self.K_lat = ctrl.place(A_lat,B_lat,p_lat)
        # print(self.K_lat[0,:])

        p_lon = np.roots([1,2*Wn_theta*h_theta,Wn_theta**2])
        self.K_lon = ctrl.place(A_lon,B_lon,p_lon)
        # print(self.K_lon[0,:])

        kr_lat = -1.0 / (C_lat*np.linalg.inv(A_lat-B_lat*self.K_lat)*B_lat)
        kr_lon = -1.0 / (C_lon*np.linalg.inv(A_lon-B_lon*self.K_lon)*B_lon)
        self.kr_lat = kr_lat[0,0]
        self.kr_lon = kr_lon[0.0]

        # state-space integrator
        # A1_lat = np.mat([[0,0,1,0,0],[0,0,0,1,0],[0,0,0,0,0],[C1,0,0,0,0],[0,-1,0,0,0]])
        # B1_lat = np.mat([[0],[0],[1.0/Jx],[0],[0]])
        # lat1_poly = np.convolve(np.convolve([1,2*Wn_phi*h_phi,Wn_phi**2],[1,2*Wn_psi*h_psi,Wn_psi**2]),[1,0.2])
        # p1_lat = np.roots(lat1_poly)
        #
        # K1_lat = ctrl.place(A1_lat,B1_lat,p1_lat)
        # self.Ki_lat = K1_lat[0,0:-1]

        A1_lon = np.mat([[0,1,0],[0,0,0],[-1.0,0,0]])
        B1_lon = np.mat([[0],[C2],[0]])

        lon1_poly = np.convolve([1,2*Wn_theta*h_theta,Wn_theta**2],[1,.35])
        p1_lon = np.roots(lon1_poly)
        K_lon = ctrl.place(A1_lon,B1_lon,p1_lon)
        self.K_lon = K_lon[0,:-1]
        self.ki_lon = K_lon[0,-1]


        # Roll Gains
        self.P_phi_ = Jx * Wn_phi**2
        self.I_phi_ = 0.0 # Probably no integrator here (slows tr and affects outer loop)
        self.D_phi_ = Jx*(2*h_phi*Wn_phi)
        self.In_phi = 0.0
        self.prev_phi = 0.0

        # Pitch Gains
        self.theta_r = 0.0
        self.P_theta_ = Wn_theta**2/b0
        self.I_theta_ = 0.35
        self.D_theta_ = 2.0*h_theta*Wn_theta/b0
        self.prev_theta = 0.0
        self.In_theta = 0.0

        # Yaw Gains
        self.psi_r = 0.0
        self.P_psi_ = Wn_psi**2 / b_psi
        self.I_psi_ = 0.02
        self.D_psi_ = 2*h_psi*Wn_psi / b_psi
        self.prev_psi = 0.0
        # self.In_psi = 0.0


        self.prev_time = rospy.Time.now()
        self.Dn_theta_prev = 0.0
        self.Dn_phi_prev = 0.0
        self.Dn_psi_prev = 0.0
        self.En_theta_prev = 0.0
        self.En_phi_prev = 0.0
        self.En_psi_prev = 0.0
        self.In_theta_prev = 0.0
        self.In_phi_prev = 0.0
        self.In_psi_prev = 0.0

        self.error_lon_d1 = 0.0
        self.error_lat_d1 = 0.0

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

        tau = .075
        Dn_theta = (2*tau - dt)/(2*tau + dt) * self.Dn_theta_prev + (2/(2*tau+dt))*(theta-self.prev_theta)
        Dn_phi = (2*tau - dt)/(2*tau + dt) * self.Dn_phi_prev + (2/(2*tau+dt))*(phi-self.prev_phi)
        Dn_psi = (2*tau - dt)/(2*tau + dt) * self.Dn_psi_prev + (2/(2*tau+dt))*(psi-self.prev_psi)

        self.Dn_theta_prev = Dn_theta
        self.Dn_phi_prev = Dn_phi
        self.Dn_psi_prev = Dn_psi
        self.prev_theta = theta
        self.prev_psi = psi
        self.prev_phi = phi

        error_lon = self.theta_r - theta
        # error_lat = self.psi_r - psi

        In_theta = self.In_theta_prev + (dt/2)*(error_lon + self.error_lon_d1)
        # In_psi = self.In_psi_prev + (dt/2)*(error_lat + self.error_lat_d1)
        self.In_theta_prev = In_theta
        self.error_lon_d1 = error_lon
        # self.In_psi_prev = In_psi

        # state vector
        x_lon = np.mat([[theta],[Dn_theta]])
        x_lat = np.mat([[phi],[psi],[Dn_phi],[Dn_psi]])

        F_tilde = -self.K_lon*x_lon + self.kr_lon*self.theta_r
        Tau_tilde = -self.K_lat*x_lat + self.kr_lat*self.psi_r

        # F_tilde = -self.K_lon*x_lon - self.ki_lon*self.In_theta
        # Tau_tilde = -self.K_lat*x_lat - self.ki_lat*self.In_psi

        F = self.Fe*np.cos(theta) + F_tilde[0,0]
        Tau = Tau_tilde[0,0]

        # this is for PID control
        # F,Tau = self.PID(phi,theta,psi,dt)

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

        # Anti-windup setup
        # if not self.I_theta_ == 0:
        #     F_sat = (l_out + r_out)*km
        #     Tau_sat = km * (l_out - r_out)*d
        #     In_theta += dt/self.I_theta_*(F_sat - F)
        # if not self.ki_lon == 0:
        #     F_sat = (l_out + r_out)*km
        #     Tau_sat = km * (l_out - r_out)*d
        #     In_theta += dt/self.ki_lon*(F_sat - F)

        self.In_theta_prev = In_theta
        # self.In_phi_prev = In_phi
        # self.In_psi_prev = In_psi

        # Pack up and send command
        command = Command()
        command.left_motor = l_out
        command.right_motor = r_out
        self.command_pub_.publish(command)

    def PID(phi,theta,psi,dt):
        # Calculate Derivative
        tau = .075
        Dn_theta = (2*tau - dt)/(2*tau + dt) * self.Dn_theta_prev + (2/(2*tau+dt))*(theta-self.prev_theta)
        Dn_phi = (2*tau - dt)/(2*tau + dt) * self.Dn_phi_prev + (2/(2*tau+dt))*(phi-self.prev_phi)
        Dn_psi = (2*tau - dt)/(2*tau + dt) * self.Dn_psi_prev + (2/(2*tau+dt))*(psi-self.prev_psi)


        # Calculate Errors and Integrals
        En_theta = self.theta_r - theta
        En_psi = self.psi_r - psi

        In_theta = self.In_theta_prev + dt/2 * (En_theta + self.En_theta_prev)
        In_psi = self.In_psi_prev + dt/2 * (En_psi + self.En_psi_prev)

        # phi_r = self.P_psi_*En_psi + self.I_psi_*In_psi - self.D_psi_*Dn_psi
        phi_r = self.P_psi_*En_psi + self.I_psi_*In_psi - self.D_psi_*(psi - self.prev_psi)/dt

        En_phi = phi_r - phi
        In_phi = self.In_phi_prev + dt/2 * (En_phi + self.En_phi_prev)


        # Calculate force
        F_tilde = self.P_theta_*En_theta + self.I_theta_*In_theta - self.D_theta_*Dn_theta
        # F_tilde = self.P_theta_*En_theta + self.I_theta_*In_theta - self.D_theta_*(theta - self.prev_theta)/dt
        F = F_tilde + self.Fe*np.cos(theta) # update the equilibrium force with the current theta

        # Calculate torque, remember that Tau_equilibrium = 0 so Tau = Tau_tilde
        # Tau = self.P_phi_*En_phi + self.I_phi_*In_phi - self.D_phi_*Dn_phi
        Tau = self.P_phi_*En_phi + self.I_phi_*In_phi - self.D_phi_*(phi - self.prev_phi)/dt


        # Update states
        self.prev_theta = theta
        self.prev_psi = psi
        self.prev_phi = phi
        self.Dn_theta_prev = Dn_theta
        self.En_theta_prev = En_theta
        self.En_phi_prev = En_phi
        self.En_psi_prev = En_psi

        return F,Tau


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass
