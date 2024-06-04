import numpy as np
from scipy.linalg import block_diag


#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################

def process_model(x):
    """
    @param x: state vector
    @return x_pred: predicted state vector
    """
    x_pred = np.zeros((3,1)) # placeholder
    # x_pred = ...
    return x_pred


def measurement_model_1(K_f, p, c):
    """
    @param  K_f: camera intrinisic matrix
    @param  p:   point
    @param  c:   optical center
    @return z:   measurement
    """
    z = np.zeros((2,1)) # placeholder
    # z = ...
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
    return z


def measurement_Jacobain_1(K_f, p):
    """
    @param K_f: intrinisic Camera 1 Matrix
    @param p: point in frame 1
    @return mesaurment jacobian
    """
    H1 = np.zeros((2,3)) # placeholder
    # H1 = ...
    return H1


def measurement_Jacobain_2(K_f, p, R, t):
    """
    @param K_f: intrinisic Camera 2 Matrix
    @param p: point in frame 1
    @param R: Rotation matrix from frame 1 to 2
    @param t: translation from frame 1 to 2
    @return mesaurment jacobian
    """
    H2 = np.zeros((2,3)) # placeholder
    # H2 = ...
    return H2

#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################

class extended_kalman_filter:

    def __init__(self, data):

        # Constants
        self.F    = np.eye(3)                          # State Transition Jacobian
        self.R1   = np.cov(data['z_1'], rowvar=False)  # measurement noise covariance
        self.R2   = np.cov(data['z_2'], rowvar=False)  # measurement noise covariance
        self.Q   = np.array([[0.03, 0.02, 0.01],
                            [0.02, 0.04, 0.01],
                            [0.01, 0.01, 0.05]])       # process noise covariance (vibrations)
        self.W    = np.eye(3)                          # process noise Jacobian
        self.C_1  = data['C_1']
        self.C_2  = data['C_2']
        self.Kf_1 = data['Kf_1']
        self.Kf_2 = data['Kf_2']
        self.Rot  = data['R']
        self.t    = data['t']
        
        # stack the noise covariances for part b
        self.R_stack = block_diag(self.R1, self.R2)
        
        # Functions
        self.f       = process_model          # process model
        self.h1      = measurement_model_1    # measurement model for sensor 1
        self.h2      = measurement_model_2    # measurement model for sensor 2
        self.H1      = measurement_Jacobain_1 # measurement Jacobian for sensor 1
        self.H2      = measurement_Jacobain_2 # measurement Jacobian for sensor 2

        # States
        self.x       = np.array([[0.12], [0.09], [1.5]]) # state vector
        self.Sigma   = np.eye(3)                         # state covariance


    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    def prediction(self):
        """
        @note: Use `self.f` instead of `process_model`
        """
        # Taking care of the motion model in camera 1 here
        # self.x = ...
        
        # predicted state covariance
        # self.Sigma = ...


    def correction1(self, z):
        """
        @note: Use `self.h1` and `self.H1` instead of `measurement_model_1` and
               `measurement_Jacobain_1`
        @param z: measurement
        """
        # get the predicted measurements
        # z_hat = ...
        # H = ...

        # innovation
        # v = ...

        # innovation covariance
        # S = ...

        # filter gain
        # K = ...

        # correct the predicted state
        # self.x = ...
        # self.Sigma = ...

    def correction2(self, z):
        """
        @note: Use `self.h2` and `self.H2` instead of `measurement_model_2` and
               `measurement_Jacobain_2`
        @param z: measurement
        """
        # get the predicted measurements
        # z_hat = ...
        # H = ...

        # innovation
        # v = ...

        # innovation covariance
        # S = ...

        # filter gain
        # K = ...

        # correct the predicted state
        # self.x = ...
        # self.Sigma = ...


    def correction_batch(self, z_stack):
        """
        @note: Use `self.h1`, `self.H1`, `self.h2` and `self.H2` here
        @params z_stack: stacked measurements
        """
        # get the predicted measurements
        # z_hat1 = ...
        # z_hat2 = ...
        # z_hat_stack = ... (4x1)

        # stacked Jacobian
        # H1 = ...
        # H2 = ...
        # H =  ... (4x3)

        # innovation
        # v = ...

        # innovation covariance
        # S = ...

        # filter gain
        # K = ...

        # correct the predicted state
        # self.x = ...
        # self.Sigma = ...

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################

def ekf_load_data():
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


def ekf_sequential(ekf, data):
    z_1 = data['z_1']
    z_2 = data['z_2']
    N = np.shape(z_1)[0]
    states = np.zeros((N+1,3,1)) # N+1 because we need to include the initial state
    states[0] = ekf.x
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    # for i in range(N):
        
        # ...

        # states[i+1] = ekf.x
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    return states


def ekf_batch(ekf, data):
    z_1 = data['z_1']
    z_2 = data['z_2']
    N = np.shape(z_1)[0]
    states = np.zeros((N+1,3,1)) # N+1 because we need to include the initial state
    states[0] = ekf.x
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    # for i in range(N):

        # ...

        # states[i+1] = ekf.x
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    return states
