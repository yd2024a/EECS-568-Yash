import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation as R

def wedge(phi):
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
    """
    Adjoint of SO3 Adjoint (R) = R
    """
    return R

#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
def motion_model(R, omega, dt):
    """
    @param  R:      State variable
    @param  omega:  gyroscope reading
    @param  dt:     time step
    @return R_pred: predicted state variable
    """
    R_pred = np.zeros((3,3)) # placeholder
    # R_pred = ...
    return R_pred


def measurement_Jacobain(g):
    """
    @param  g: gravity
    @return H: measurement Jacobain
    """
    H = np.zeros((3,3)) # placeholder
    # H = ...
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
        """
        @param omega: gyroscope reading
        @param dt:    time step
        """

        # self.X = ...

        # self.P = ...

        return

    def correction(self, Y, g):
        """
        @param Y: linear acceleration measurement
        @param g: gravity
        """

        # H = ...
        # N = ...
        # S = ...
        # L = ...

        # Update state
        # self.X = ...

        # Update Covariance
        # self.P = ...

        return

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################


def riekf_load_data():
    data = {}
    data['accel'] = np.loadtxt(open('data/a.csv'), delimiter=",")
    data['omega'] = np.loadtxt(open('data/omega.csv'), delimiter=",")
    data['dt'] = np.loadtxt(open('data/dt.csv'), delimiter=",")
    data['gravity'] = np.loadtxt(open('data/gravity.csv'), delimiter=",")
    data['euler_gt'] = np.loadtxt(open('data/euler_gt.csv'), delimiter=",")
    return data


def ahrs_riekf(iekf_filter, data):
    # useful variables
    accel = data['accel']
    omega = data['omega']
    dt = data['dt']
    gravity = data['gravity']
    N = data['accel'].shape[0]

    states_rot = np.zeros((N+1, 3, 3))
    states_rot[0] = iekf_filter.X
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    # for i in range(N):

        # ...

        # states_rot[i+1] = iekf_filter.X
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    
    # convert rotation matrices to euler angles
    states_euler = np.zeros((N+1, 3))
    for i, rot in enumerate(states_rot):
        r = R.from_matrix(rot)
        states_euler[i] = r.as_euler('zyx')
    return states_euler