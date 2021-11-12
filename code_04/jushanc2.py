import numpy as np
class RobotController:
    #this part references `raa5`
    
    def __init__(self, limiter=None):
        self.dt = 0.01
        self.limiter = limiter
        self.p_x_e = p_x_e
        self.p_y_e = p_y_e
        self.p_z_e = p_z_e
        self.phi_e = phi_e
        self.theta_e = theta_e
        self.psi_e = psi_e
        self.v_x_e = v_x_e
        self.v_y_e = v_y_e
        self.v_z_e = v_z_e
        self.w_x_e = w_x_e
        self.w_y_e = w_y_e
        self.w_z_e = w_z_e
        self.tau_x_e = tau_x_e
        self.tau_y_e = tau_y_e
        self.tau_z_e = tau_z_e
        self.f_z_e = f_z_e
        self.A = np.array([[ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  1.  ,  0.  ,  0.  ,
         0.  ,  0.  ,  0.  ],
       [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  1.  ,  0.  ,
         0.  ,  0.  ,  0.  ],
       [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  1.  ,
         0.  ,  0.  ,  0.  ],
       [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,
         1.  , -0.  ,  0.  ],
       [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,
         0.  ,  1.  ,  0.  ],
       [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,
        -0.  ,  0.  ,  1.  ],
       [ 0.  ,  0.  ,  0.  ,  9.81,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,
         0.  ,  0.  ,  0.  ],
       [ 0.  ,  0.  ,  0.  ,  0.  , -9.81,  0.  ,  0.  ,  0.  ,  0.  ,
         0.  ,  0.  ,  0.  ],
       [ 0.  ,  0.  ,  0.  , -0.  , -0.  ,  0.  ,  0.  ,  0.  ,  0.  ,
         0.  ,  0.  ,  0.  ],
       [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,
         0.  , -0.  , -0.  ],
       [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,
         0.  ,  0.  ,  0.  ],
       [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,
         0.  ,  0.  ,  0.  ]])
        self.B = np.array([[  0.       ,   0.       ,   0.       ,   0.       ],
       [  0.       ,   0.       ,   0.       ,   0.       ],
       [  0.       ,   0.       ,   0.       ,   0.       ],
       [  0.       ,   0.       ,   0.       ,   0.       ],
       [  0.       ,   0.       ,   0.       ,   0.       ],
       [  0.       ,   0.       ,   0.       ,   0.       ],
       [  0.       ,   0.       ,   0.       ,   0.       ],
       [  0.       ,   0.       ,   0.       ,  -0.       ],
       [  0.       ,   0.       ,   0.       ,   2.       ],
       [434.7826087,   0.       ,   0.       ,   0.       ],
       [  0.       , 434.7826087,   0.       ,   0.       ],
       [  0.       ,   0.       , 250.       ,   0.       ]])
        self.C = np.array([[1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
       [0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
       [0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
       [0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.],
       [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
       [0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.]])
        self.K = np.array([[ 6.59380473e-01,  1.60534092e-14, -1.07465565e-15,
         1.73195859e+00, -1.23883249e-14,  1.57849817e-16,
         6.70983580e-01,  8.58517395e-15, -1.28861957e-15,
         2.26815499e-01, -1.00116757e-16,  2.46410746e-17],
       [ 3.95983053e-15, -1.04257207e+00, -1.74343352e-15,
        -8.79470106e-15,  1.67354335e+00,  6.12226404e-16,
        -2.59611690e-15, -6.31818634e-01, -3.49256295e-16,
        -1.00116757e-16,  2.26222369e-01, -8.89770196e-17],
       [ 1.27081967e-15,  5.35804445e-16,  5.67172187e-16,
         1.25368799e-16,  5.39937850e-17,  2.08514414e-01,
         1.66545657e-16,  8.77687954e-17,  1.86208208e-15,
         1.41686179e-17, -5.11617863e-17,  2.12476766e-01],
       [ 1.94796628e-17, -1.96781109e-16,  9.32504808e-01,
        -1.71969650e-16,  2.83326749e-17,  2.67847575e-16,
        -4.33296394e-19,  4.11742867e-18,  1.07233209e+00,
        -5.92765004e-18, -1.60657896e-18,  1.48966566e-17]])
        self.L = np.array([[ 2.75129226e+01,  0.00000000e+00,  0.00000000e+00,
         1.85098263e-01,  0.00000000e+00,  0.00000000e+00],
       [ 0.00000000e+00,  2.75129226e+01,  0.00000000e+00,
         0.00000000e+00, -1.85098263e-01,  0.00000000e+00],
       [ 0.00000000e+00,  0.00000000e+00,  2.74392971e+01,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
       [ 2.15947973e-01,  0.00000000e+00,  0.00000000e+00,
         2.54744934e+01,  0.00000000e+00,  0.00000000e+00],
       [ 0.00000000e+00, -2.15947973e-01,  0.00000000e+00,
         0.00000000e+00,  2.54744934e+01,  0.00000000e+00],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  2.54752781e+01],
       [ 2.85004418e+01,  0.00000000e+00,  0.00000000e+00,
         9.79490615e+00,  0.00000000e+00,  0.00000000e+00],
       [ 0.00000000e+00,  2.85004418e+01,  0.00000000e+00,
         0.00000000e+00, -9.79490615e+00,  0.00000000e+00],
       [ 0.00000000e+00,  0.00000000e+00,  2.64575131e+01,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
       [ 1.51345988e-02,  0.00000000e+00,  0.00000000e+00,
         2.44948934e+01,  0.00000000e+00,  0.00000000e+00],
       [ 0.00000000e+00, -1.51345988e-02,  0.00000000e+00,
         0.00000000e+00,  2.44948934e+01,  0.00000000e+00],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  2.44948974e+01]])

    def get_color(self):
        return [0., 1., 0.]

    def reset(self, pos):
        self.xhat = np.zeros((12,1))
        self.t = 0
        
    def run(self, pos, rpy, pos_ring, is_last_ring, pos_others):
        pxdes = pos_ring[0]
        pydes = pos_ring[1]
        pzdes = pos_ring[2]
        
        unit_vec = abs(pos_ring-pos)/np.linalg.norm(pos_ring-pos)
        if pos[0] < 2.5 and pos[1] < 2.5:
            pzdes = 2
            
        pos_estimate = np.array([self.xhat[0],self.xhat[1],self.xhat[2]]).T[0]
        error = pos_ring-pos_estimate
        
        if (abs(error[1])) > 3 or (abs(error[0]) ) > 2.5:
#             pxdes  += 1*unit_vec[0]
#             pydes += 2*unit_vec[1]
#             pzdes += 2*unit_vec[2]
            pxdes = pos_estimate[0]+(5*error[0]/np.linalg.norm(error))
            pydes = pos_estimate[1]+(5*error[1]/np.linalg.norm(error))
            pzdes = pos_estimate[2]+(8*error[2]/np.linalg.norm(error))
        else:
            pxdes = pos_estimate[0]+(3*error[0]/np.linalg.norm(error))
            pydes = pos_estimate[1]+(3*error[1]/np.linalg.norm(error))
            pzdes = pos_estimate[2]+(10*error[2]/np.linalg.norm(error))
        
        if abs(error[1]) > 0.65:
            pxdes = pxdes-1.675
            
        if (self.t<3 and pos_ring[2]<1):
            pzdes = 1

        if is_last_ring==True:
            pzdes = 2

        xhatdes = np.array([[pxdes - p_x_e],[pydes - p_y_e],[pzdes - p_z_e],[0],[0],[0],[0],[0],[0],[0],[0],[0]])
        
        u = -self.K@(self.xhat - xhatdes)#from  clss
        
        tau_x = u[0,0]+self.tau_x_e
        tau_y = u[1,0]+self.tau_y_e
        tau_z = u[2,0]+self.tau_z_e
        f_z =  u[3,0]+self.f_z_e
        
        y = np.concatenate((pos, rpy), axis=None) - np.array([p_x_e,p_y_e,p_z_e,phi_e,theta_e,psi_e])
        y = (y.reshape((6,1))).astype(float)
        
        if self.limiter is not None:
            tau_x, tau_y, tau_z, f_z = self.limiter(tau_x, tau_y, tau_z, f_z)
        
        self.xhat += self.dt*(self.A@self.xhat + self.B@u - self.L@(self.C@self.xhat - y))
        if (is_last_ring == True and np.linalg.norm(error)<3):
            return  0,0,0,0
        return tau_x, tau_y, tau_z, f_z