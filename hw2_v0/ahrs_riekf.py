import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation as R
from scipy.linalg import block_diag, expm

def wedge(phi):
    # No modifications needed.
    print("A")
    """
    R^3 vector to so(3) matrix
    @param  phi: R^3
    @return Phi: so(3) matrix
    """
    phi = phi.squeeze()
    Phi = np.array([[0, -phi[2], phi[1]],
                    [phi[2], 0, -phi[0]],
                    [-phi[1], phi[0], 0]])
    return Phi

def adjoint(R):
    # No modifications needed.
    print("B")
    """
    Adjoint of SO3 Adjoint (R) = R
    """
    return R

#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
def motion_model(R1, omega, dt):
    # Derive and implement this.
    print("C")
    """
    @param  R:      State variable
    @param  omega:  gyroscope reading
    @param  dt:     time step
    @return R_pred: predicted state variable
    """
    print("R: ", R1)
    print("omega: ", omega)
    print("dt: ", dt)
    print("")
    R_pred = np.zeros((3,3)) # placeholder
    # R_pred = ...
        # Convert omega to radians per second
    omega_rad = np.radians(omega)
    
    # Create a rotation object from the current orientation matrix R
    #rot_current = R.from_matrix(R.as_matrix())
    rot_current = R.from_matrix(R1)
    #rot_current = R(R)

    # Convert the angular velocity vector to an axis-angle representation
    axis_angle = omega_rad * dt
    
    # Apply rotation over time using Rodrigues' formula
    rot_pred = rot_current * R.from_rotvec(axis_angle)
    
    # Extract the predicted orientation matrix
    R_pred = rot_pred.as_matrix()
    print("R_pred: ", R_pred)
    return R_pred


def measurement_Jacobain(g):
    # Derive and implement this.
    print("D")
    """
    @param  g: gravity
    @return H: measurement Jacobain
    """
    print("g: ", g)
    print("")
    H = np.zeros((3,3)) # placeholder
    # H = ...
    Xi = np.array([1,0,0])
    b = g
    # H = ...
    Xi_hat = wedge(Xi)
    print("Xi_hat: ", Xi_hat)
    #H = np.dot(Xi_hat,b)
    H = Xi_hat*b
    print("H: ", H)
    return H

#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################

class right_iekf:

    def __init__(self):
        self.Phi = np.eye(3)               # state transtion matrix
        self.Q = 1e-4*np.eye(3)            # gyroscope noise covariance
        self.N = 1e-4*np.eye(3)            # accelerometer noise covariance
        self.f = motion_model              # process model
        self.H = measurement_Jacobain      # measurement Jacobain
        self.X = np.eye(3)                 # state vector
        self.P = 0.1 * np.eye(3)           # state covariance

    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    def prediction(self, omega, dt):
        # Implement
        print("Debug E")
        """
        @param omega: gyroscope reading
        @param dt:    time step
        """

        self.X = self.f(self.X, omega, dt)

        self.P = self.P

        return

    def correction(self, Y, g):
        # Implement
        print("Debug F")
        """
        @param Y: linear acceleration measurement
        @param g: gravity
        """

        H = self.H
        N = np.dot(np.dot(self.X, block_diag(self.N, 0), self.X.T))
        S = np.dot(np.dot(H,self.P), H.T) + N
        L = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        # Update state
        nu = np.dot(block_diag(self.X,self.X),Y)-H
        delta = wedge(np.dot(L,nu))
        self.X = np.dot(expm(delta),self.X)

        # Update Covariance
        I = np.eye(np.shape(self.P)[0])
        temp = I - np.dot(L,H)
        self.P = np.dot(np.dot(temp,self.P), temp.T) + np.dot(np.dot(L,N), L.T)

        return

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################


def riekf_load_data():
    # No modifications necessary
    print("Debug G")
    data = {}
    data['accel'] = np.loadtxt(open('data/a.csv'), delimiter=",")
    data['omega'] = np.loadtxt(open('data/omega.csv'), delimiter=",")
    data['dt'] = np.loadtxt(open('data/dt.csv'), delimiter=",")
    data['gravity'] = np.loadtxt(open('data/gravity.csv'), delimiter=",")
    data['euler_gt'] = np.loadtxt(open('data/euler_gt.csv'), delimiter=",")
    print("euler_gt shape: ", data['euler_gt'].shape)
    #print("")
    return data


def ahrs_riekf(iekf_filter, data):
    # Implement (partially)
    print("Debug H")
    # useful variables
    accel = data['accel']
    print("accel shape: ", accel.shape)
    omega = data['omega']
    print("omega shape: ", omega.shape)
    dt = data['dt']
    print("dt shape: ", dt.shape)
    gravity = data['gravity']
    print("gravity shape: ", gravity.shape)
    N = data['accel'].shape[0]
    print("N: ", N)

    states_rot = np.zeros((N+1, 3, 3))
    print("states_rot shape: ", states_rot.shape)
    states_rot[0] = iekf_filter.X
    print("states_rot[0]: ", states_rot[0])
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    # Remember, N = 1277, so this loop is going to run that many times.
    for i in range(N):

        # ...
        R_pred = iekf_filter.f(states_rot[i], omega[i], dt[i])
        H = iekf_filter.H(gravity)
        states_rot[i+1] = iekf_filter.X
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    
    # convert rotation matrices to euler angles
    states_euler = np.zeros((N+1, 3))
    print("states_euler shape: ", states_euler.shape)
    print("enumerate(states_rot): ", enumerate(states_rot))
    for i, rot in enumerate(states_rot):
        #print("")
        # print("i: ", i)
        # i begins at 0, goes up to 1277
        # print("rot: ", rot)
        # rot is a 3x3 matrix, full of zeros.
        r = R.from_matrix(rot)
        #print("r", r.shape)
        #Shape is not applicable for R
        states_euler[i] = r.as_euler('zyx')
        #print("states_euler[i]: ", states_euler[i].shape)
        # Expect the above debug statement to print...
        # about 1277 1x3 matrices.
    return states_euler
