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
    x_pred = x
    return x_pred
sample_x = np.zeros((3,1))
print("")
print("Debug process_model: ", process_model(sample_x))
print("")

def measurement_model_1(K_f, p, c):
    """
    @param  K_f: camera intrinisic matrix
    @param  p:   point
    @param  c:   optical center
    @return z:   measurement
    """
    z = np.zeros((2,1)) # placeholder
    # Debug statements to print data types
    #print("Data type of K_f:", type(K_f))
    #print("Data type of p:", type(p))
    #print("Data type of c:", type(c))
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
        #print("z:", z)
    return z
# Define placeholder values
sample_K_f = np.array([[1,0],[0,1]])
sample_p = np.array([[4],[5],[6]])
sample_c = np.array([[2],[2]])
# Call the function with placeholder values
print("")
result1 = measurement_model_1(sample_K_f, sample_p, sample_c)
print("result1: ", result1)
print("")
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
    p2 = np.transpose(R) @ p - np.transpose(R) @ t
    #print("np.transpose(R): ", np.transpose(R))
    #print("p: ", p)
    #print("t: ", t)
    #print("p2: ", p2)
    # p2 = np.array([[3, 1, 2], [4, 5, 7], [8, 9, 6]])
    p_x_2 = p2[0,0]
    p_y_2 = p2[1,0]
    p_z_2 = p2[2,0]

    #print("p_x_2: ", p_x_2)
    #print("p_y_2: ", p_y_2)
    #print("p_z_2: ", p_z_2)

    #print("K_f: ", K_f)
    #print("np.array([[p2[0,0]],[p2[1,0]]]): ", np.array([[p2[0,0]],[p2[1,0]]]))
    #print("c: ", c)
    # This is based on a WILD GUESS that Kf_1 and Kf_2 are equal. (i.e. K_f = K_f_2, or something like that).
    if p_z_2 == 0:
        #print("Debug B")
        print("Warning. Division by zero encountered. Setting measurement to NaN")
        z = np.nan
    else:
        z = K_f @ np.array([[p_x_2/p_z_2],[p_y_2/p_z_2]]) + c
        #print("z measurement model 2:", z)
    # z = ...
    return z
# Define placeholder values
sample_K_f = np.array([[1,0],[0,1]])
sample_p = np.array([[4],[5],[6]])
sample_c = np.array([[2],[2]])
#sample_R = np.array([[1]])
sample_R = np.eye(3)
sample_t = np.array([[1],[0],[0]])

# Call the function with placeholder values
print("")
result2 = measurement_model_2(sample_K_f, sample_p, sample_c,sample_R,sample_t)
print("result2: ", result2)
print("")

def measurement_Jacobain_1(K_f, p):
    """
    @param K_f: intrinisic Camera 1 Matrix
    @param p: point in frame 1
    @return mesaurment jacobian
    """
    H1 = np.zeros((2,3)) # placeholder
    p_x = p[0,0]
    p_y = p[1,0]
    p_z = p[2,0]
    #print("HI THERE!!! K_f shape: ", K_f.shape)
    if p_z == 0:
        print("Warning. Division by zero encountered. Setting measurement to NaN")
        H1 = np.nan
    else:
        H1 = K_f @ np.array([[1/p_z, 0, -p_x/(p_z ** 2)],
                          [0, 1/p_z, -p_y/(p_z ** 2)]])
        #print("Measurement Jacobian 1:", H1)
    # H1 = ...
    return H1
# Define placeholder values
#sample_K_f = np.array([[1,0,0],[0,1,1]]) # SIZE RE-ADJUSTED. Not sure why this was necessary. See previous K_f values in debug statements.
sample_K_f = np.array([[1,0],[0,1]]) # Never mind, this worked! Ignore the "sample_K_f in the previous line".
sample_p = np.array([[4],[5],[6]])
#print("A")
result3 = measurement_Jacobain_1(sample_K_f, sample_p)
#print("B")
def measurement_Jacobain_2(K_f, p, R, t):
    """
    @param K_f: intrinisic Camera 2 Matrix
    @param p: point in frame 1
    @param R: Rotation matrix from frame 1 to 2
    @param t: translation from frame 1 to 2
    @return mesaurment jacobian
    """
    H2 = np.zeros((2,3)) # placeholder
    #print("Shape of K_f: ", K_f.shape)
    #print("Shape of p: ", p.shape)
    #print("Shape of R: ", R.shape)
    #print("Shape of t: ", t.shape)
    #print("K_f: ", K_f)
    #print("p: ", p)
    #print("R: ", R)
    #print("t: ", t)
    t_1, t_2, t_3 = t[0,0], t[1,0], t[2,0]
    # p is a 3x1 column vector, R is a transformation matrix, and t_1, t_2, t_3 are scalar values
    p_x, p_y, p_z = p[0,0], p[1,0], p[2,0]

    # q_x = R*p_x + t_1
    # q_y = R*p_y + t_2
    # q_z = R*p_z + t_3

    q_x = R[0, 0] * p_x + R[0, 1] * p_y + R[0, 2] * p_z + t_1
    q_y = R[1, 0] * p_x + R[1, 1] * p_y + R[1, 2] * p_z + t_2
    q_z = R[2, 0] * p_x + R[2, 1] * p_y + R[2, 2] * p_z + t_3
    #print("q_z: ", q_z)
    if q_z == 0:
        print("Warning. Division by zero encountered. Setting measurement to NaN")
        H2 = np.nan
    else:
        #testing = np.array([[1/q_z, 0, -q_x/(q_z ** 2)], [0, 1/q_z, -q_y/(q_z ** 2)]])
        #print("Shape of testing: ", testing.shape)
        H2 = K_f @ np.array([[1/q_z, 0, -q_x/(q_z ** 2)], [0, 1/q_z, -q_y/(q_z ** 2)]]) @ np.transpose(R) 
        #print("Measurement Jacobian 2:", H2)       
    #H2 = K_f*np.aray
    # H2 = ...
    return H2

# Define placeholder values
# sample_K_f = np.array([[1,0,0],[0,1,1]]) # SIZE RE-ADJUSTED. Not sure why this was necessary. See previous K_f values in debug statements.
sample_K_f = np.array([[1,0],[0,1]])
sample_p = np.array([[4],[5],[6]])
sample_R = np.eye(3)
sample_t = np.array([[1],[0],[0]])

# Call the function with placeholder values
print("")
result4 = measurement_Jacobain_2(sample_K_f, sample_p, sample_R, sample_t)
print("result4: ", result4)
print("")
#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################
    # NON-PRINT DEBUG BEGINS
    # Define the data dictionary
"""
data = {
    'z_1': np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),  # Example measurement data for sensor 1
    'z_2': np.array([[9, 8, 7], [6, 5, 4], [3, 2, 1]]),  # Example measurement data for sensor 2
    'C_1': np.array([[318.6], [255.3]]),  # Example data for C_1 (you can fill in with actual data if needed)
    'C_2': np.array([[325.1], [249.7]]),  # Example data for C_2 (you can fill in with actual data if needed)
    'Kf_1': np.array([[517.3, 0], [0, 516.5]]),  # Example data for Kf_1 (you can fill in with actual data if needed)
    'Kf_2': np.array([[520.9, 0], [0, 521]]),  # Example data for Kf_2 (you can fill in with actual data if needed)
    'R': np.eye(3),  # Example data for R (you can fill in with actual data if needed)
    't': np.array([[1],[2],[0]])  # Example data for t (you can fill in with actual data if needed)
    }
    # NON-PRINT DEBUG ENDS.
"""

class extended_kalman_filter:

    def __init__(self, data):

        # Constants
        self.F    = np.eye(3)                          # State Transition Jacobian
        self.R1   = np.cov(data['z_1'], rowvar=False)  # measurement noise covariance
        #print("HAPPY FEBRUARY!")
        #print("self.R1: ", self.R1)
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
        #print("HAPPY TUESDAY!")
        #print("self.t: ", self.t)
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
    #print("Debug A1")
    
    def prediction(self):
        """
        @note: Use `self.f` instead of `process_model`
        """
        # Debugging print statements
        #print("Debug A2")
        #print("self.x:", self.x)
        #print("self.F:", self.F)
        #print("np.transpose(self.F):", np.transpose(self.F))
        #print("self.W:", self.W)
        #print("self.Q:", self.Q)
        #print("np.transpose(self.W):", np.transpose(self.W))
        # Taking care of the motion model in camera 1 here
        self.x = self.f(self.x)
        #print("self.x: ", self.x)
        # predicted state covariance
        self.Sigma = self.F @ self.Sigma @ np.transpose(self.F) + self.W @ self.Q @ np.transpose(self.W)
        #print("self.Sigma: ", self.Sigma)
        #print("")
    #print("Debug A3")


    def correction1(self, z):
        """
        @note: Use `self.h1` and `self.H1` instead of `measurement_model_1` and
               `measurement_Jacobain_1`
        @param z: measurement
        """

        print("Debug C1")
        print("Input measurement data (z): ", z)
        # get the predicted measurements
        #p = np.array([[4],[5],[6]])
        #c = np.array([[2],[2]])
        """
        p = self.x
        c = C_1
        """
        #K_f = np.array([[1,0],[0,1]])
        #z_hat = self.h1(K_f, p, c) # Predicted Measurements
        z_hat = self.h1(self.Kf_1, self.x, self.C_1) # Predicted Measurements
        #print("Debug z_hat: ", z_hat)
        #print("Debug self.x: ", self.x)

        #print("Debug: self.Kf_1: ", self.Kf_1)
        #print("Debug: p: ", p)
        #H = self.H1(K_f, p) # Measurement Jacobian
        H = self.H1(self.Kf_1, self.x)
        #print("Debug H: ", H)
        # innovation
        #print("Debug z: ", z)
        #print("Debug z_hat: ", z_hat)
        v = z-z_hat
        #print("v: ", v)
        # innovation covariance
        print("Debug H: ", H)
        print("Debug self.Sigma: ", self.Sigma)
        print("Debug np.transpose(H): ", np.transpose(H))
        print("Debug self.R1: ", self.R1)
        S = H @ self.Sigma @ np.transpose(H) + self.R1[:2,:2]
        #print("S: ", S)
        # filter gain
        K = self.Sigma @ np.transpose(H) @ np.linalg.inv(S) # Kalman Gain
        #print("K: ", K)
        # correct the predicted state
        
        self.x = self.x + K @ v # correct the predicted state
        print("correction 1: self.x: ", self.x)
        #print("self.x: ", self.x)
        print("len(self.x): ", len(self.x))
        I2 = np.eye(len(self.x)) # Added Line: Update the covariance matrix, part 1
        #print("I2: ", I2)
        self.Sigma = (I2 - K @ H) @ self.Sigma # Update the covariance matrix, part 2
        print("self.Sigma: ", self.Sigma)


    def correction2(self, z):
        """
        @note: Use `self.h2` and `self.H2` instead of `measurement_model_2` and
               `measurement_Jacobain_2`
        @param z: measurement
        """
        #print("Debug D1")
        #print("Input measurement data (z), under correction2: ", z)
        # Dummy Variables
        # Note: z to be 3x1 matrix from resoning in def correction1(self, z)
        #K_f = np.array([[1,0],[0,1]])
        """
        K_f was originally 2x2. Re-shaped to avoid addition error when computing innovation covariance.
        c was reshaped as well, see below.
        """
        #K_f = np.array([[1,0],[0,1]])
        #K_f = np.array([[1,0],[0,1]])
        #p = np.array([[4],[5],[6]])
        #c = np.array([[2],[2]])
        #c_adjusted = np.array([[2],[2],[0]])
        #R = np.eye(3)
        #t = np.array([[1],[0],[0]])

        # get the predicted measurements
        #z_hat = self.h2(K_f,p,c,R,t)
        z_hat = self.h2(self.Kf_2,self.x,self.C_2,self.R2,self.t)
        #print("z_hat correction2: ", z_hat)
        #H = self.H2(K_f,p,R,t) # Measurement Jacobian
        H = self.H2(self.Kf_2,self.x,self.R2,self.t)
        #print("H correction2: ", H)
        # innovation
        v = z-z_hat
        #print("")
        #print("v: ", v)

        # innovation covariance
        #print("Debug H: ", H)
        #print("Debug self.Sigma: ", self.Sigma)
        #print("Debug np.transpose(H): ", np.transpose(H))
        #print("Debug self.R2: ", self.R2)
        S = H @ self.Sigma @ np.transpose(H) + self.R2[:2,:2]

        # filter gain
        K = self.Sigma @ np.transpose(H) @ np.linalg.inv(S)
        #print("K: ", K)
        #print("self.x: ", self.x)
        # correct the predicted state
        self.x = self.x + K @ v # correct the predicted state
        #print("correction2: self.x: ", self.x)
        I2 = np.eye(len(self.x)) # Added Line: Update the covariance matrix, part 1
        self.Sigma = (I2 - K @ H) @ self.Sigma # Update the covariance matrix, part 2
        #print("")
        #print("self.Sigma: ", self.Sigma)


    def correction_batch(self, z_stack):
        """
        @note: Use `self.h1`, `self.H1`, `self.h2` and `self.H2` here
        @params z_stack: stacked measurements
        """
        """
        Below are added dummy variables. Feel free to get rid of them
        as necessary, or change them. see correction1 function for first
        three dummy variables K_f, p, and c. See correction2 function
        for next two.
        """
        #K_f = np.array([[1,0],[0,1]])
        #p = np.array([[4],[5],[6]])
        #c = np.array([[2],[2]])
        #R = np.eye(3)
        #t = np.array([[1],[0],[0]])
        # get the predicted measurements
        #z_hat1 = self.h1(K_f, p, c) # Predicted Measurements
        z_hat1 = self.h1(self.Kf_1,self.x,self.C_1)
        #z_hat2 = self.h2(K_f,p,c,R,t)
        z_hat2 = self.h2(self.Kf_2,self.x,self.C_2,self.R2,self.t)
        print("")
        print("z_hat1: ", z_hat1)
        print("z_hat2: ", z_hat2)
        z_hat_stack = np.vstack((z_hat1,z_hat2)) #(4x1)

        # stacked Jacobian
        #H1 = self.H1(K_f, p) # Measurement Jacobian
        H1 = self.H1(self.Kf_1,self.x)
        #H2 = self.H2(K_f,p,R,t) # Measurement Jacobian
        H2 = self.H2(self.Kf_2,self.x,self.R2,self.t)
        H =  np.vstack((H1,H2)) # (4x3)
        print("H stacked: ", H)

        # innovation
        v = z_stack-z_hat_stack
        print("correction batch, v: ", v)
        # innovation covariance
        """
        Dimensional mismatch betweeen the terms being added below.
        4x4 versus 3x3 matrices.
        """
        print("")
        print("correction_batch, H @ self.Sigma @ np.transpose(H): ", H @ self.Sigma @ np.transpose(H))
        print("correction_batch, self.R2: ", self.R2)
        R2_adjusted = np.zeros((4,4), dtype=self.R2.dtype)
        R2_adjusted[:3,:3] = self.R2
        R2_adj = R2_adjusted
        print("R2_adjusted: ", R2_adjusted)
        #S = H @ self.Sigma @ np.transpose(H) + self.R2
        S = H @ self.Sigma @ np.transpose(H) + R2_adj
        print("correction batch, S: ",S)
        # filter gain
        # K = ...
        K = self.Sigma @ np.transpose(H) @ np.linalg.inv(S)
        print("correction batch, K: ", K)
        print("correction batch, self.x: ", self.x)
        print("correction batch, K @ v: ", K@v)

        # correct the predicted state
        self.x = self.x + K @ v
        print("correction batch, self.x (post update): ", self.x)
        I2 = np.eye(len(self.x)) # Added Line: Update the covariance matrix, part 1
        print("K @ H: ", K @ H)
        print("self.Sigma: initial: ", self.Sigma)
        self.Sigma = (I2 - K @ H) @ self.Sigma # Update the covariance matrix, part 2
        print("")
        print("self.Sigma: ", self.Sigma)

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
        ekf.correction1(z_1[i].reshape((2,1)))
        ekf.correction2(z_1[i].reshape((2,1)))
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
    for i in range(N):

        # ...
        z_stack = np.vstack((z_1,z_2)).reshape((4,1))
        ekf.prediction()
        ekf.correction_batch(z_stack)
        states[i+1] = ekf.x
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    return states

"""
# NON-PRINT DEBUG, PART 1b: START
# Create an instance of the extended_kalman_filter class
ekf_instance = extended_kalman_filter(data)

# Call the prediction method on the instance
ekf_instance.prediction()
# NON-PRINT DEBUG, PART 2: END.
"""
"""
Non-print debug statements contd., see below.
"""
"""
ekf_instance = extended_kalman_filter(ekf_load_data)
sample = np.array([[1],[1],[1]])
# Call the prediction method on the instance
print("Debug A1")
ekf_instance.prediction()
print("ekf_instance.prediction(): ", ekf_instance.prediction())
print("Debug A2")

# Debugging the correction1 function call
ekf_instance = extended_kalman_filter(data)
z_example = np.array([[389.32], [297.51]])
ekf_instance.correction1(z_example)
#print("Debug C2")

#print("")
#print("")
#print("")

# Debugging the correction2 function call
ekf_instance2 = extended_kalman_filter(data)
z_example2 = np.array([[389.32], [297.51]])
ekf_instance2.correction2(z_example2)
#print("Debug D2")

ekf_instance_stack = extended_kalman_filter(data)
z_example_stack = np.vstack((z_example,z_example2))
ekf_instance_stack.correction_batch(z_example_stack)
"""



