import numpy as np
from numpy.random import randn, rand
from scipy.stats import multivariate_normal
from scipy.linalg import block_diag
import matplotlib.pyplot as plt # added line.

#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
def process_model(x, w):
    """
    Think this makes sense because we don't move at all
    @param  x:      actually point with respect to frame 1
    @param  w:      noise vector sampled as part of the motion model
    @return x_pred: predicted state vector
    """
    #print("process_model pf: x: ", x)
    #print("process_model pf: w: ", w)
    

    x_pred = np.zeros((3,1)) # placeholder
    # x_pred = ...
    x_pred = x.reshape(3,1) + w
    #x_pred = x_pred.reshape(3,1)
    print("x_pred: ", x_pred)
    print("")
    return x_pred

def measurement_model_1(K_f, p, c):
    """
    @param  K_f: camera intrinisic matrix
    @param  p:   point
    @param  c:   optical center
    @return z:   measurement
    """
    print("measurement_model_1: K_f: ", K_f)
    print("measurement_model_1: p: ", p)
    print("measurement_model_1: c: ", c)
    z = np.zeros((2,1)) # placeholder
    # z = ...
    p_x = p[0,0]
    p_y = p[1,0]
    p_z = p[2,0]

    # Avoiding division by zero:
    if p_z == 0:
        #print("Debug A")
        print("Warning. Division by zero encountered. Setting measurement to NaN")
        z = np.nan
    else:
        z = K_f @ np.array([[p_x/p_z],[p_y/p_z]]) + c
    print("z: ", z)
    print("")
    return z

def measurement_model_2(K_f, p, c, R, t):
    """
    @param  K_f: camera intrinisic matrix
    @param  p:   point
    @param  c:   optical center
    @param  R:   rotation matrix of camera 2 wrt camera 1
    @param  t:   translation vector of camera 2 wrt camera 1
    @return z:   measurement
    """
    z = np.zeros((2,1)) # placeholder
    # z = ...
    if np.transpose(R).shape == (2, 2):
    # Pad np.transpose(R) with zeros to make it shape (3, 3)
        R_padded = np.pad(np.transpose(R), ((0, 1), (0, 1)), mode='constant')
    else:
        R_padded = np.transpose(R)

# Now R_padded should have shape (3, 3), either original shape or padded with zeros
    #p2 = np.transpose(R) @ p - np.transpose(R) @ t
    p2 = R_padded @ p - R_padded @ t 
    p_x_2 = p2[0,0]
    p_y_2 = p2[1,0]
    p_z_2 = p2[2,0]
    print("p_z_2: ", p_z_2)

    if p_z_2 == 0:
        print("Warning. Division by zero encountered. Setting measurement to NaN")
        z = np.nan
    else:
        z = K_f @ np.array([[p_x_2/p_z_2],[p_y_2/p_z_2]]) + c
    return z

#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################
class particle_filter:
    """
    Particle filter class for state estimation of a nonlinear system
    The implementation follows the Sample Importance Resampling (SIR)
    filter a.k.a bootstrap filter
    """

    def __init__(self, data, N = 1000):

        # Constants
        self.R1 = np.cov(data['z_1'], rowvar=False)  # measurement noise covariance
        self.R2 = np.cov(data['z_2'], rowvar=False)  # measurement noise covariance
        self.N  = N # number of particles
        self.Q = np.array([[0.03,0.02,0.01],
                            [0.02,0.04, 0.01],
                            [0.01,0.01, 0.05]]).reshape((3,3)) # input noise covariance (vibrations)
        self.LQ = np.linalg.cholesky(self.Q)
        self.C_1  = data['C_1']
        self.C_2  = data['C_2']
        self.Kf_1 = data['Kf_1']
        self.Kf_2 = data['Kf_2']
        self.Rot  = data['R']
        self.t    = data['t']
        
        # stack the noise covariances for part b
        self.R_stack = block_diag(self.R1, self.R2)

        # Functions
        self.f  = process_model        # process model
        self.h1 = measurement_model_1  # measurement model
        self.h2 = measurement_model_2  # measurement model

        # initialize particles
        self.p = {}
        self.p['x'] = np.zeros((self.N, 3))       # particles
        self.p['w'] = 1/self.N * np.ones(self.N)  # importance weights

        # initial state
        self.x_init  = np.array([0.12, 0.09, 0.5]) # state vector
        self.Sigma_init = np.eye(3)                # state covariance
        self.N_eff = 0

        L_init = np.linalg.cholesky(self.Sigma_init)
        for i in range(self.N):
            self.p['x'][i] = (L_init @ randn(3, 1)).squeeze() + self.x_init

    
    def resampling(self):
        """
        low variance resampling
        """
        W = np.cumsum(self.p['w'])
        r = rand(1) / self.N
        j = 1
        for i in range(self.N):
            u = r + (i - 1) / self.N
            while u > W[j]:
                j = j + 1
            self.p['x'][i, :] = self.p['x'][j, :]
            self.p['w'][i] = 1 / self.N

    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    def sample_motion(self):
        """
        random walk motion model
        @note: Use `self.f` instead of `process_model`
        """
        for i in range(self.N):
            # sample noise
            # do not change this line
            w = self.LQ @ randn(3, 1)

            # propagate the particle
            print("w: ", w)
            print("self.p['x'][i, :]: ", self.p['x'][i, :])
            print("pf sample motion: self.f(self.p['x'][i, :], w): ", self.f(self.p['x'][i, :], w))
            self.p['x'][i, :] = self.f(self.p['x'][i, :], w).reshape(1, 3)

    def importance_measurement_1(self, z):
        """
        @param z: stacked measurement vector 2x1
        @note: Use `self.h1` instead of `measurement_model_1`
        """
        # compute importance weights
        print("z.shape: ", z.shape)
        #print("pf importance_measurement_1: ", w)        
        # compute importance weights
        w = np.zeros(self.N)
        print("w.shape: ", w.shape)
        #print("w: ", w)
        for i in range(self.N):
            # compute innovation
            v = z[i][:,] - self.h1(self.Kf_1, self.p['x'][i, :], self.C_1 )
            # Above: modified from original: v = z - self.h(self.p.x[i, :])
            ##w[i] = multivariate_normal.pdf(v.reshape(-1), np.array([0, 0]), self.R1) 
            # Above: Try self.Rot if this doesn't work

        # update and normalize weights
        ##self.p['w'] = np.multiply(self.p['w'], w)  # since we used motion model to sample
        ##self.p['w'] = self.p['w'] / np.sum(self.p['w'])

        # compute effective number of particles
        ##self.N_eff = 1 / np.sum(np.power(self.p['w'], 2))  # effective number of particles
        # Above: Original: 1 / np.sum(np.power(self.p.w, 2))  # effective number of particles
        print("")

    def importance_measurement_2(self, z):
        """
        @param z: stacked measurement vector 2x1
        @note: Use `self.h2` instead of `measurement_model_2`
        """
        # compute importance weights
        print("pf importance_measurement_2: z: ", z)
        #print("pf importance_measurement_2: ", w)
        w = np.zeros(self.N)
        for i in range(self.N):
            # compute innovation
            v = z - self.h2(self.p['x'][i, :])
            ##w[i] = multivariate_normal.pdf(v.reshape(-1), np.array([0, 0]), self.R2)
            # Above: Try self.Rot if this doesn't work

        # update and normalize weights
        ##self.p['w'] = np.multiply(self.p['w'], w)  # since we used motion model to sample
        ##self.p['w'] = self.p['w'] / np.sum(self.p['w'])

        # compute effective number of particles
        ##self.N_eff = 1 / np.sum(np.power(self.p['w'], 2))  # effective number of particles
        ##print("")

    def importance_mesurement_batch(self, z):
        """
        @param z: stacked measurement vector 4x1
        @note: Use `self.h1` instead of `measurement_model_1`
        @note: Use `self.h2` instead of `measurement_model_2`
        """
        print("print importance_measurement_batch: z: ", z)
        # Split the measurement vector into two parts
        z_1 = z[:2]
        z_2 = z[2:]
        # compute importance weights
        w = np.zeros(self.N)
        for i in range(self.N):
            # compute innovation for measurement 1
            v1 = z_1 - self.h1(self.p['x'][i, :])
            # compute innovation for measurement 2
            v2 = z_2 - self.h2(self.p['x'][i, :])
            # compute innovation (may not be needed, will double-check)
            #     v = ...
            # compute weights for both measurements and combine
            w1 = multivariate_normal.pdf(v1.reshape(-1), np.array([0, 0]), self.R1)
            w2 = multivariate_normal.pdf(v2.reshape(-1), np.array([0, 0]), self.R2)
            w[i] = w1 * w2

        # update and normalize weights
        self.p['w'] = np.multiply(self.p['w'], w)
        self.p['w'] = self.p['w'] / np.sum(self.p['w'])

        # compute effective number of particles
        self.N_eff = 1 / np.sum(np.power(self.p['w'], 2))

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################