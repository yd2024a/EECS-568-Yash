import numpy as np
from scipy.linalg import block_diag


#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################

def process_model(x):
    """
    @param x: state vector (3, 1)
    @return x_pred: predicted state vector (3, 1)
    """
    x_pred = np.zeros((3,1)) # placeholder
    # x_pred = ...
    x_pred = x
    return x_pred


def measurement_model_1(K_f, p, c):
    """
    @param  K_f: camera intrinisic matrix (2, 2)
    @param  p:   point (3, 1)
    @param  c:   optical center (2, 1)
    @return z:   measurement (2, 1)
    """
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
    return z


def measurement_model_2(K_f, p, c, R, t):
    """
    @param  K_f: camera intrinisic matrix (2, 2)
    @param  p:   point (3, 1)
    @param  c:   optical center (2, 1)
    @param  R:   rotation matrix of camera 2 wrt camera 1 (3, 3)
    @param  t:   translation vector of camera 2 wrt camera 1 (3, 1)
    @return z:   measurement (2, 1)
    """
    z = np.zeros((2,1)) # placeholder
    print("measurement model 2, np.transpose(R): ", R.T)
    #print("measurement model 2, np.transpose(R).shape: ", R.T.shape)
    #print("measurement model 2, p: ", p)
    #print("measurement model 2, p.shape: ", p.shape)
    #print("measurement model 2, t: ", t)
    #print("measurement model 2, t.shape: ", t.shape)

    # Assuming np.transpose(R) has shape (2, 2)
    """
    if np.transpose(R).shape == (2, 2):
    # Pad np.transpose(R) with zeros to make it shape (3, 3)
        R_padded = np.pad(np.transpose(R), ((0, 1), (0, 1)), mode='constant')
    else:
        R_padded = np.transpose(R)
    """
    # Now R_padded should have shape (3, 3), either original shape or padded with zeros
    #p2 = np.transpose(R) @ p - np.transpose(R) @ t
    #p2 = R_padded @ p - R_padded @ t 
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
    # z = ...
    return z

def measurement_Jacobain_1(K_f, p):
    """
    @param K_f: intrinisic Camera 1 Matrix (2, 2)
    @param p: point in frame 1 (3, 1)
    @return mesaurment jacobian (2, 3)
    """
    H1 = np.zeros((2,3)) # placeholder
    p_x = p[0,0]
    p_y = p[1,0]
    p_z = p[2,0]

    if p_z == 0:
        print("Warning. Division by zero encountered. Setting measurement to NaN")
        H1 = np.nan
    else:
        H1 = K_f @ np.array([[1/p_z, 0, -p_x/(p_z ** 2)],
                          [0, 1/p_z, -p_y/(p_z ** 2)]])
        print("measurement_Jacobian_1, H1: ", H1)
    # H1 = ...
    return H1


def measurement_Jacobain_2(K_f, p, R, t):
    """
    @param K_f: intrinisic Camera 2 Matrix (2, 2)
    @param p: point in frame 1 (3, 1)
    @param R: Rotation matrix from frame 1 to 2 (3, 3)
    @param t: translation from frame 1 to 2 (3, 1)
    @return mesaurment jacobian (2, 3)
    """
    H2 = np.zeros((2,3)) # placeholder
    # H2 = ...
    t_1, t_2, t_3 = t[0,0], t[1,0], t[2,0]
    p_x, p_y, p_z = p[0,0], p[1,0], p[2,0]

    print(" measurement Jacobian 2, R: ", R)
    """
    print("measurement Jacobian 2, p_x: ", p_x)
    print("measurement Jacobian 2, p_y: ", p_y)
    print("measurement Jacobian 2, p_z: ", p_z)
    print("measurement Jacobian 2, t_1: ", t_1)
    print("measurement Jacobian 2, t_2: ", t_2)
    print("measurement Jacobian 2, t_3: ", t_3)
    
    if np.transpose(R).shape == (2, 2):
    # Pad np.transpose(R) with zeros to make it shape (3, 3)
        R_padded = np.pad(np.transpose(R), ((0, 1), (0, 1)), mode='constant')
    else:
        R_padded = np.transpose(R)
    R = R_padded
    print("R adjusted: ", R)    
    """

    #q_x = R[0, 0] * p_x + R[0, 1] * p_y + R[0, 2] * p_z + t_1
    #q_y = R[1, 0] * p_x + R[1, 1] * p_y + R[1, 2] * p_z + t_2
    #q_z = R[2, 0] * p_x + R[2, 1] * p_y + R[2, 2] * p_z + t_3

    q_x = R.T[0,0] * (p_x-t_1) + R.T[0,1] * (p_y-t_2) + R.T[0,2] * (p_z-t_3)
    q_y = R.T[1,0] * (p_x-t_1) + R.T[1,1] * (p_y-t_2) + R.T[1,2] * (p_z-t_3)
    q_z = R.T[2,0] * (p_x-t_1) + R.T[2,1] * (p_y-t_2) + R.T[2,2] * (p_z-t_3)

    if q_z == 0:
        print("Warning. Division by zero encountered. Setting measurement to NaN")
        H2 = np.nan
    else:
        H2 = K_f @ np.array([[1/q_z, 0, -q_x/(q_z ** 2)], [0, 1/q_z, -q_y/(q_z ** 2)]]) @ np.transpose(R)
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
        print("self.Rot: ", self.Rot)
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
        self.x = self.f(self.x)
        
        # predicted state covariance
        # self.Sigma = ...
        self.Sigma = self.F @ self.Sigma @ np.transpose(self.F) + self.W @ self.Q @ np.transpose(self.W)


    def correction1(self, z):
        """
        @note: Use `self.h1` and `self.H1` instead of `measurement_model_1` and
               `measurement_Jacobain_1`
        @param z: measurement
        """
        # get the predicted measurements
        # z_hat = ...
        z_hat = self.h1(self.Kf_1, self.x, self.C_1) # Predicted Measurements
        # H = ...
        H = self.H1(self.Kf_1, self.x)

        # innovation
        # v = ...
        v = z-z_hat

        # innovation covariance
        # S = ...
        S = H @ self.Sigma @ np.transpose(H) + self.R1[:2,:2]

        # filter gain
        # K = ...
        K = self.Sigma @ np.transpose(H) @ np.linalg.inv(S) # Kalman Gain

        # correct the predicted state
        # self.x = ...
        self.x = self.x + K @ v # correct the predicted state
        # self.Sigma = ...
        I2 = np.eye(len(self.x)) # Added Line: Update the covariance matrix, part 1
        self.Sigma = (I2 - K @ H) @ self.Sigma # Update the covariance matrix, part 2

    def correction2(self, z):
        """
        @note: Use `self.h2` and `self.H2` instead of `measurement_model_2` and
               `measurement_Jacobain_2`
        @param z: measurement
        """
        # get the predicted measurements
        # z_hat = ...
        z_hat = self.h2(self.Kf_2,self.x,self.C_2,self.Rot,self.t)
        # H = ...
        H = self.H2(self.Kf_2,self.x,self.Rot,self.t)

        # innovation
        # v = ...
        v = z-z_hat

        # innovation covariance
        # S = ...
        print("H: ", H)
        print("self.sigma: ", self.Sigma)
        print("H @ self.Sigma @ np.transpose(H): ", H @ self.Sigma @ np.transpose(H))
        print("self.R2[:2,:2]: ", self.R2[:2,:2])
        S = H @ self.Sigma @ np.transpose(H) + self.R2[:2,:2]
        #S = H @ self.Sigma @ np.transpose(H) + self.Rot
        # filter gain
        # K = ...
        K = self.Sigma @ np.transpose(H) @ np.linalg.inv(S)

        # correct the predicted state
        # self.x = ...
        self.x = self.x + K @ v # correct the predicted state
        I2 = np.eye(len(self.x)) # Added Line: Update the covariance matrix, part 1
        # self.Sigma = ...
        self.Sigma = (I2 - K @ H) @ self.Sigma # Update the covariance matrix, part 2


    def correction_batch(self, z_stack):
        """
        @note: Use `self.h1`, `self.H1`, `self.h2` and `self.H2` here
        @params z_stack: stacked measurements
        """
        print("")
        print("")
        print("")
        print("correction_batch, self.Kf_2: ", self.Kf_2)
        print("correction_batch, self.x: ", self.x)
        print("correction_batch, self.C_2: ", self.C_2)
        print("correction_batch, self.R2: ", self.R2)
        print("correction_batch, self.C_2: ", self.t)

        # get the predicted measurements
        # z_hat1 = ...
        z_hat1 = self.h1(self.Kf_1,self.x,self.C_1)
        print("correction_batch, z_hat1: ", z_hat1)
        # z_hat2 = ...
        z_hat2 = self.h2(self.Kf_2,self.x,self.C_2,self.Rot,self.t)
        #print("correction_batch, z_hat2: ". z_hat2)
        # z_hat_stack = ... (4x1)
        z_hat_stack = np.vstack((z_hat1,z_hat2)) #(4x1)
        #print("correction_batch, z_hat_stack: ". z_hat_stack)

        # stacked Jacobian
        # H1 = ...
        H1 = self.H1(self.Kf_1,self.x)
        print("self.x: ", self.x)
        print("self.Kf_1: ", self.Kf_1)
        print("correction_batch, H1: ", H1)
        # H2 = ...
        H2 = self.H2(self.Kf_2,self.x,self.Rot,self.t)
        print("correction_batch, H2: ", H2)
        # H =  ... (4x3)
        H =  np.vstack((H1,H2)) # (4x3)
        print("correction_batch, H: ", H)

        # innovation
        # v = ...
        print("correction_batch, z_stack: ", z_stack)
        print("correction_batch, z_hat_stack: ", z_hat_stack)
        v = z_stack-z_hat_stack
        #print("correction_batch, v: ", v)
        """
        R2_adjusted = np.zeros((4,4), dtype=self.R2.dtype)
        print("correction batch, R2_adjusted: ", R2_adjusted)
        print("correction batch, self.R: ", self.Rot)
        R2_adjusted[:3,:3] = self.R2
        R2_adj = R2_adjusted
        """
        # innovation covariance
        # S = ...
        S = H @ self.Sigma @ np.transpose(H) + self.R_stack

        # filter gain
        # K = ...
        K = self.Sigma @ np.transpose(H) @ np.linalg.inv(S)

        # correct the predicted state
        # self.x = ...
        self.x = self.x + K @ v
        I2 = np.eye(len(self.x)) # Added Line: Update the covariance matrix, part 1
        # self.Sigma = ...
        print("(I2 - K @ H): ", I2 - K @ H)
        print("self.Sigma: ", self.Sigma)
        self.Sigma = (I2 - K @ H) @ self.Sigma # Update the covariance matrix, part 2

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
    for i in range(N):
        
        # ...
        ekf.prediction()
        ekf.correction1(z_1[i][:, np.newaxis])
        ekf.correction2(z_2[i][:, np.newaxis])
        states[i+1] = ekf.x
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
    stacked_transposes = []
    for i in range(N):

        # ...

        #z_stack = 
        #z_stack = z_stack[:, np.newaxis]
        print("z_1[i]: ", z_1[i])
        print("z_2[i]: ", z_2[i])
        ekf.prediction()
        stacked_transposes = np.vstack((z_1[i].reshape(-1,1), z_2[i].reshape(-1,1)))
        print("stacked_transposes: ", stacked_transposes)
        ekf.correction_batch(stacked_transposes)
        #stacked_transposes.append(stacked_transposes)
        #ekf.correction_batch(z_stack)
        states[i+1] = ekf.x
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    return states
