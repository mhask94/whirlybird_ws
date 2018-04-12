import numpy as np
import control as ctrl

g = 9.81
l1 = 0.85
l2 = 0.3048
m1 = 0.891
m2 = 1.0
d = 0.178
h = 0.65
r = 0.12
Jx = 0.0047
Jy = 0.0014
Jz = 0.0041
km = 5.440583
Fe = g/l1*(m1*l1-m2*l2)

# Check for controllability
C1 = l1*Fe / (m1*l1**2+m2*l2**2+Jz)
C2 = l1 / (m1*l1**2+m2*l2**2+Jy)

A_lat = np.mat([[0,0,1,0],[0,0,0,1],[0,0,0,0],[C1,0,0,0]])
B_lat = np.mat([[0],[0],[1.0/Jx],[0]])

C_ab_lat = ctrl.ctrb(A_lat,B_lat)
rank_lat = np.linalg.matrix_rank(C_ab_lat)
print('lat rank = %d',rank_lat)
if rank_lat != 4:
    print('Warning: Not Controllable')

A_lon = np.mat([[0,1],[0,0]])
B_lon = np.mat([[0],[C2]])

C_ab_lon = ctrl.ctrb(A_lon,B_lon)
rank_lon = np.linalg.matrix_rank(C_ab_lon)
print('lon rank = ',rank_lon)
if rank_lon != 2:
    print('Warning: Not Controllable')

# state-space equilibrium values
self.phi_e = 0.0
self.theta_e = 0.0
self.psi_e = 0.0
self.r_lat_e = 0.0
self.r_lon_e = 0.0
self.u_lat_e = 0.0
self.u_lon_e = Fe

self.x_lat_e = np.mat([[self.phi_e],self.psi_e],[0],[0])
self.x_lon_e = np.mat([[self.theta_e],[0]])

p = np.mat([[]])
self.K_lat = ctrl.place(A_lat,B_lat,p)
