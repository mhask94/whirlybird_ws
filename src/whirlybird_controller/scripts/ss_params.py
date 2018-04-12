import numpy as np
import control as ctrl

class SSControl:
    def __init__(self):
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
        C_lat = np.mat([[1.,0,0,0]])

        C_ab_lat = ctrl.ctrb(A_lat,B_lat)
        rank_lat = np.linalg.matrix_rank(C_ab_lat)
        print('lat rank = ',rank_lat)
        if rank_lat != 4:
            print('Warning: Not Controllable')

        A_lon = np.mat([[0,1],[0,0]])
        B_lon = np.mat([[0],[C2]])
        C_lon = np.mat([[1.,0]])

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

        self.x_lat_e = np.mat([[self.phi_e],[self.psi_e],[0],[0]])
        self.x_lon_e = np.mat([[self.theta_e],[0]])

        R1 = -1.0
        I1 = 1.0j
        R2 = -1.5
        I2 = 1.5j
        # p = np.array([[R1+I1],[R1-I1],[R2+I2],[R2-I2]])
        p_lat = [R1+I1,R1-I1,R2+I2,R2-I2]
        self.K_lat = ctrl.place(A_lat,B_lat,p_lat)
        print(self.K_lat[0,:])

        R = -1.0
        I = 1.0j
        # p = np.array([[R1+I1],[R1-I1],[R2+I2],[R2-I2]])
        p_lon = [R+I,R-I]
        self.K_lon = ctrl.place(A_lon,B_lon,p_lon)
        print(self.K_lon[0,:])

        self.kr_lat = -1.0 / (C_lat*np.linalg.inv(A_lat-B_lat*self.K_lat)*B_lat)
        self.kr_lon = -1.0 / (C_lon*np.linalg.inv(A_lon-B_lon*self.K_lon)*B_lon)
        print('kr values:')
        print(self.kr_lat[0,0])
        print(self.kr_lon[0,0])


def main():
        controller = SSControl()

if __name__ == '__main__':
    main()
