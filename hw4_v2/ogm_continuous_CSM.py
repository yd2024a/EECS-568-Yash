# Occupancy Grid Mapping Counting Sensor Model Class
#
# Author: Chien Erh Lin, Fangtong Liu, Peter Stratton
# Date: 03/09/2024

import numpy as np
from scipy.spatial import KDTree
from tqdm import tqdm

def continuous_CSM(map, pose, m_i, sigma, l, z, i, k):
    bearing_diff = []
    # find the nearest beam
    bearing_diff = np.abs(wrapToPI(z[:, 1] - m_i['phi']))
    # idx = np.nanargmin(bearing_diff)
    indexes = np.argpartition(bearing_diff, 1)

    for idx in indexes[:1]:
        global_x = pose['x'][k][0] + z[idx,0] * np.cos(z[idx,1] + pose['h'][k][0])
        global_y = pose['y'][k][0] + z[idx,0] * np.sin(z[idx,1] + pose['h'][k][0])

        # -----------------------------------------------
        # To Do: 
        # implement the continuous counting sensor model, update 
        # map['alpha'] and map['beta']
        #
        # Hint: use distance and l to determine occupied or free.
        # There might be multiple ways to update map['beta']. 
        # One way is to segment the measurement into several range 
        # values and update map['beta'] if the distance is smaller 
        # than l  
        # -----------------------------------------------
        # update end point -> occupied
        # 
        # Hint: use step size = l * (2/3) when finding sample points along the beam
        minDist = float('inf')
        difference = map['occMap'].data[i, :] - [global_x, global_y]
        #print("Debug difference: ", difference)
        d1 = np.linalg.norm(difference) # Norm 2 needs to be comuted. This is norm1?
        if(d1 < l):
            kernel = sigma*((1/3)*(2+np.cos(2*np.pi*(d1/l)))*(1-(d1/l))+(1/(2*np.pi))*np.sin(2*np.pi*(d1/l)))
            map['alpha'][i] = map['alpha'][i]+kernel
        else:
            kernel = 0
            step_size = l * (2/3)
            for r in np.arange(0, z[idx,0], step_size):
                global_x2 = pose['x'][k][0] + r * np.cos(z[idx,1] + pose['h'][k][0])
                global_y2 = pose['y'][k][0] + r * np.sin(z[idx,1] + pose['h'][k][0])
                difference_2 = map['occMap'].data[i, :] - [global_x2,global_y2]
                d2 = np.linalg.norm(difference_2)
                if (d2 < minDist):
                    minDist = d2
            if (minDist < l):
                kernel_updated = sigma*((1/3)*(2+np.cos(2*np.pi*(minDist/l)))*(1-(minDist/l))+(1/(2*np.pi))*np.sin(2*np.pi*(minDist/l)))
                map['beta'][i] = map['beta'][i] + kernel_updated

# Occupancy Grid Mapping Class
class ogm_continuous_CSM:

    def __init__(self, grid_size = 0.135):
        # map dimensions
        self.range_x = [-15, 20]
        self.range_y = [-25, 10]

        # senesor parameters
        self.z_max = 30     # max range in meters
        self.n_beams = 133  # number of beams, we set it to 133 because not all measurements in the dataset contains 180 beams 

        # grid map parameters
        self.grid_size = grid_size              # adjust this for task 2.B
        self.nn = 16                            # number of nearest neighbor search

        # map structure
        self.map = {}   # map
        self.pose = {}  # pose data
        self.scan = []  # laser scan data
        self.m_i = {}   # cell i

        # continuous csm function 
        self.continuous_CSM = continuous_CSM

        # continuous kernel parameter
        self.l = 0.2      # kernel parameter
        self.sigma = 0.1  # kernel parameter

        # -----------------------------------------------
        # To Do: 
        # prior initialization
        # Initialize prior, prior_alpha
        # -----------------------------------------------
        self.prior = 1e-6            # prior for setting up mean and variance
        self.prior_alpha = 1e-6       # a small, uninformative prior for setting up alpha

    def construct_map(self, pose, scan):
        # class constructor
        # construct map points, i.e., grid centroids
        x = np.arange(self.range_x[0], self.range_x[1]+self.grid_size, self.grid_size)
        y = np.arange(self.range_y[0], self.range_y[1]+self.grid_size, self.grid_size)
        X, Y = np.meshgrid(x, y)
        t = np.hstack((X.reshape(-1, 1), Y.reshape(-1, 1)))

        # a simple KDTree data structure for map coordinates
        self.map['occMap'] = KDTree(t)
        self.map['size'] = t.shape[0]

        # set robot pose and laser scan data
        self.pose['x'] = pose['x'][0][0]
        self.pose['y'] = pose['y'][0][0]
        self.pose['h'] = pose['h'][0][0]
        self.pose['mdl'] = KDTree(np.hstack((self.pose['x'], self.pose['y'])))
        self.scan = scan

        # -----------------------------------------------
        # To Do: 
        # Initialization map parameters such as map['mean'], map['variance'], map['alpha'], map['beta']
        # -----------------------------------------------
        self.map['mean'] = np.ones((self.map['size'], 1))*self.prior       # size should be (number of data) x (1)
        self.map['variance'] = np.ones((self.map['size'], 1))*self.prior   # size should be (number of data) x (1)
        self.map['alpha'] = np.ones((self.map['size'], 1))*self.prior_alpha      # size should be (number of data) x (1)
        self.map['beta'] =  np.ones((self.map['size'], 1))*self.prior_alpha      # size should be (number of data) x (1) 


    def is_in_perceptual_field(self, m, p):
        # check if the map cell m is within the perception field of the
        # robot located at pose p
        inside = False
        d = m - p[0:2].reshape(-1)
        self.m_i['range'] = np.sqrt(np.sum(np.power(d, 2)))
        self.m_i['phi'] = wrapToPI(np.arctan2(d[1], d[0]) - p[2])
        # check if the range is within the feasible interval
        if (0 < self.m_i['range']) and (self.m_i['range'] < self.z_max):
            # here sensor covers -pi to pi
            if (-np.pi < self.m_i['phi']) and (self.m_i['phi'] < np.pi):
                inside = True
        return inside

    def build_ogm(self, disable_print=False):
        # build occupancy grid map using the binary Bayes filter.
        # We first loop over all map cells, then for each cell, we find
        # N nearest neighbor poses to build the map. Note that this is
        # more efficient than looping over all poses and all map cells
        # for each pose which should be the case in online (incremental)
        # data processing.
        for i in tqdm(range(self.map['size']), disable=disable_print):
            m = self.map['occMap'].data[i, :]
            _, idxs = self.pose['mdl'].query(m, self.nn)
            if len(idxs):
                for k in idxs:
                    # pose k
                    pose_k = np.array([self.pose['x'][k], self.pose['y'][k], self.pose['h'][k]])
                    if self.is_in_perceptual_field(m, pose_k):
                        # laser scan at kth state; convert from cartesian to
                        # polar coordinates
                        z = cart2pol(self.scan[k][0][0, :], self.scan[k][0][1, :])
                        # -----------------------------------------------
                        # To Do: 
                        # update the sensor model in cell i
                        # -----------------------------------------------
                        self.continuous_CSM(self.map, self.pose, self.m_i, self.sigma, self.l, z, i, k)

            # -----------------------------------------------
            # To Do: 
            # update mean and variance for each cell i
            # -----------------------------------------------
            self.map['mean'][i] = (self.map['alpha'][i])/(self.map['alpha'][i]+self.map['beta'][i])
            self.map['variance'][i] = (self.map['alpha'][i]*self.map['beta'][i])/((self.map['alpha'][i]+self.map['beta'][i])**2  * (self.map['alpha'][i] + self.map['beta'][i] + 1))


# This function is used to convert Cartesian to Polar
def cart2pol(x, y):
    r = np.sqrt(x ** 2 + y ** 2)
    theta = np.arctan2(y, x)
    z = np.hstack((r.reshape(-1, 1), theta.reshape(-1, 1)))
    return z


# This function is used to wrap angles in radians to the interval [-pi, pi]
# pi maps to pi and -pi maps to -pi
def wrapToPI(phase):
    x_wrap = np.remainder(phase, 2 * np.pi)
    idx = np.argwhere(np.abs(x_wrap) > np.pi)
    while len(idx):
        x_wrap[idx] -= 2 * np.pi * np.sign(x_wrap[idx])
        idx = np.argwhere(np.abs(x_wrap) > np.pi)
    return x_wrap