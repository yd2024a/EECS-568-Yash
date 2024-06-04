import numpy as np
from numpy.random import randn, rand
from scipy.stats import multivariate_normal
from scipy.linalg import block_diag


#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
def process_model(x, w):
    print("Debug A")
    """
    @param  x:      point with respect to frame 1 (3,)
    @param  w:      noise vector sampled as part of the motion model (3, 1)
    @return x_pred: predicted state vector (3, 1)
    """
    print("process_model pf: x: ", x)
    print("process_model pf: w: ", w)
    x_pred = np.zeros((3,1)) # placeholder
    # x_pred = ...
    x_pred = x.reshape(3,1) + w
    #x_pred = x_pred.reshape(3,1)
    print("x_pred: ", x_pred)
    print("")
    return x_pred

def measurement_model_1(K_f, p, c):
    print("Debug B")
    """
    @param  K_f: camera intrinisic matrix (2, 2)
    @param  p:   point (3,)
    @param  c:   optical center (2, 1)
    @return z:   measurement (2, 1)
    """
    z = np.zeros((2,1)) # placeholder
    # z = ..
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
    print("Debug C")
    """
    @param  K_f: camera intrinisic matrix (2, 2)
    @param  p:   point (3,)
    @param  c:   optical center (2, 1)
    @param  R:   rotation matrix of camera 2 wrt camera 1 (3, 3)
    @param  t:   translation vector of camera 2 wrt camera 1 (3, 1)
    @return z:   measurement (2, 1)
    """
    z = np.zeros((2,1)) # placeholder
    # z = ...
    p2 = np.transpose(R) @ p - np.transpose(R) @ t
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

    def __init__(self, data, N=1000):

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
        print("Debug D")
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
        print("Debug E")
        """
        random walk motion model
        @note: Use `self.f` instead of `process_model`
        """
        for i in range(self.N):
            # sample noise
            # do not change this line
            w = self.LQ @ randn(3, 1)

            # propagate the particle
            print("")
            print("w: ", w)
            print("self.p['x'][i, :]: ", self.p['x'][i, :])
            print("pf sample motion: self.f(self.p['x'][i, :], w): ", self.f(self.p['x'][i, :], w))
            self.p['x'][i, :] = self.f(self.p['x'][i, :], w).reshape(1, 3)

    def importance_measurement_1(self, z):
        print("Debug F")
        """
        @param z: stacked measurement vector 2x1
        @note: Use `self.h1` instead of `measurement_model_1`
        """
        print("z.shape: ", z.shape)
        # compute importance weights
        w = np.zeros(self.N)
        for i in range(self.N):
        #     # compute innovation
        #     v = ...
              print("self.Kf_1: ", self.Kf_1)
              print("self.C_1: ", self.C_1)
              print("self.p['x'][i, :]: ", self.p['x'][i, :])
              print("np.transpose(self.p['x'][i, :]): ", self.p['x'][i, :])
              row_vector = np.array(self.p['x'][i, :])
              column_vector = row_vector[:, np.newaxis]
              print("row_vector: ", row_vector)
              print("column_vector: ", column_vector)
              #print("z[i]: ", z[i])
              #print("z[i][:,]: ", z[i][:,])
              v = z - self.h1(self.Kf_1, column_vector, self.C_1)
              w[i] = multivariate_normal.pdf(v.reshape(-1), np.array([0, 0]), self.R1)

        # update and normalize weights
        self.p['w'] = np.multiply(self.p['w'], w)  # since we used motion model to sample
        self.p['w'] = self.p['w'] / np.sum(self.p['w'])

        # compute effective number of particles
        self.N_eff = self.N_eff = 1 / np.sum(np.power(self.p['w'], 2))  # effective number of particles
        # Above: Original: 1 / np.sum(np.power(self.p.w, 2))  # effective number of particles

    def importance_measurement_2(self, z):
        print("Debug G")
        """
        @param z: stacked measurement vector 2x1
        @note: Use `self.h2` instead of `measurement_model_2`
        """
        # compute importance weights
        w = np.zeros(self.N)
        for i in range(self.N):
            # compute innovation
            print("self.Kf_2: ", self.Kf_2)
            print("self.C_2: ", self.C_2)
            row_vector2 = np.array(self.p['x'][i, :])
            column_vector2 = row_vector2[:, np.newaxis]
            print("row_vector2: ", row_vector2)
            print("column_vector2: ", column_vector2)
            v = z - self.h2(self.Kf_2, column_vector2, self.C_2, self.Rot, self.t)
            w[i] = multivariate_normal.pdf(v.reshape(-1), np.array([0, 0]), self.R2)

        # update and normalize weights
        self.p['w'] = np.multiply(self.p['w'], w)  # since we used motion model to sample
        self.p['w'] = self.p['w'] / np.sum(self.p['w'])

        # compute effective number of particles
        self.N_eff = 1 / np.sum(np.power(self.p['w'], 2))  # effective number of particles
        # Above: Original: 1 / np.sum(np.power(self.p.w, 2))  # effective number of particles

    def importance_mesurement_batch(self, z):
        print("Debug H")
        """
        @note: should be importance_me*a*surement_batch, for
               compatibility reasons this will not be fixed in
               this semester
        @param z: stacked measurement vector 4x1
        @note: Use `self.h1` instead of `measurement_model_1`
        @note: Use `self.h2` instead of `measurement_model_2`
        """
            # Split the measurement vector into two parts
        z_1 = z[:2]
        z_2 = z[2:]
        # compute importance weights
        w = np.zeros(self.N)
        for i in range(self.N):
            # compute innovation
            row_vector = np.array(self.p['x'][i, :])
            column_vector = row_vector[:, np.newaxis]
            print("row_vector: ", row_vector)
            print("column_vector: ", column_vector)
            v1 = z_1 - self.h1(self.Kf_1, column_vector, self.C_1)
            row_vector2 = np.array(self.p['x'][i, :])
            column_vector2 = row_vector2[:, np.newaxis]
            print("row_vector2: ", row_vector2)
            print("column_vector2: ", column_vector2)
            v2 = z_2 - self.h2(self.Kf_2, column_vector2, self.C_2, self.Rot, self.t)
            #v = np.vstack(v1,v2)
            w1 = multivariate_normal.pdf(v1.reshape(-1), np.array([0,0]), self.R1)
            w2 = multivariate_normal.pdf(v2.reshape(-1), np.array([0, 0]), self.R2)
            w[i] = w1 * w2           
            #w[i] = ...

            # block diagonal R1 and R2 



        # update and normalize weights
        # self.p['w'] = ...

        # compute effective number of particles
        # self.N_eff = ...

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################


def pf_load_data():
    print("Debug I")
    data = {}
    data['C_1'] = np.loadtxt(open('data/C_1.csv'), delimiter=",").reshape(-1, 1)
    data['C_2'] = np.loadtxt(open('data/C_2.csv'), delimiter=",").reshape(-1, 1)
    data['Kf_1'] = np.loadtxt(open('data/Kf_1.csv'), delimiter=",")
    data['Kf_2'] = np.loadtxt(open('data/Kf_2.csv'), delimiter=",")
    data['R'] = np.loadtxt(open('data/R.csv'), delimiter=",")
    data['t'] = np.loadtxt(open('data/t.csv'), delimiter=",").reshape(-1, 1)
    data['z_1'] = np.loadtxt(open('data/z_1.csv'), delimiter=",")
    data['z_2'] = np.loadtxt(open('data/z_2.csv'), delimiter=",")    
    return data

def pf_sequential(pf, data):
    print("Debug J")
    z_1 = data['z_1']
    z_2 = data['z_2']
    N = np.shape(z_1)[0]
    states = np.zeros((N+1, 3))
    N_t = pf.N/100 # resampling threshold

    states[0] = pf.x_init.squeeze()
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    print("N = ", N)
    for i in range(N):
        
        # ...
        # use N_t to determine when to resample

        # states[i+1] = ...
        pf.sample_motion()
        pf.importance_measurement_1(z_1[i][:, np.newaxis])
        if pf.N_eff < N_t:
            pf.resampling()
        pf.importance_measurement_2(z_2[i][:, np.newaxis])
        if pf.N_eff < N_t:
            pf.resampling()
        
        # Make sure to reshape z:
        x = np.sum(pf.p['x'] * pf.p['w'][:, np.newaxis], axis=0) / np.sum(pf.p['w'])
        states[i+1] = x.reshape(1,3)
        #states[i+1] = pf.x_init.squeeze()
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################

    return states

def pf_batch(pf, data):
    print("")
    print("Debug K")
    z_1 = data['z_1']
    z_2 = data['z_2']
    print("z_1.shape(): ", z_1.shape)
    print("z_2.shape(): ", z_2.shape)
    N = np.shape(z_1)[0]
    states = np.zeros((N+1, 3))
    N_t = pf.N/5 # resampling threshold

    states[0] = pf.x_init.squeeze()
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    for i in range(N):
        pf.sample_motion()
        # ...
        # use N_t to determine when to resample
        #z_batch = np.vstack((z_1[i], z_2[i]))
        #z_batch = np.vstack((z_1[i].reshape(-1), z_2[i].reshape(-1)))
        #z_batch = np.concatenate((z_1[i, :].reshape([2,1]), z_2[i, :].reshape([2,1])) ,axis=0)
        z_batch = np.vstack((z_1[i].reshape(2, 1), z_2[i].reshape(2, 1)))
        pf.importance_mesurement_batch(z_batch)

        if pf.N_eff < N_t:
            pf.resampling()

        x = np.sum(pf.p['x'] * pf.p['w'][:, np.newaxis], axis=0) / np.sum(pf.p['w'])
        states[i+1] = x
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
            
    return states
