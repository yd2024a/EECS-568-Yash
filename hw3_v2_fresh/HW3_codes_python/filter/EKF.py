import numpy as np
from scipy.linalg import block_diag
from copy import deepcopy, copy

from system.RobotState import RobotState
from utils.Landmark import LandmarkList
from utils.utils import wrap2Pi

class EKF:

    def __init__(self, system, init):
        # EKF Construct an instance of this class
        # Inputs:
        #   system: system and noise models
        #   init:   initial state mean and covariance
        self.gfun = system.gfun  # motion model
        self.hfun = system.hfun  # measurement model
        self.Gfun = init.Gfun  # Jocabian of motion model
        self.Vfun = init.Vfun  # Jocabian of motion model
        self.Hfun = init.Hfun  # Jocabian of measurement model
        self.M = system.M # motion noise covariance
        self.Q = system.Q # measurement noise covariance

        self.state_ = RobotState()

        # init state
        self.state_.setState(init.mu)
        self.state_.setCovariance(init.Sigma)


    ## Do prediction and set state in RobotState()
    def prediction(self, u, X, P, step):
        if step == 0:
            X = self.state_.getState()
            P = self.state_.getCovariance()
        else:
            X = X
            P = P
        ###############################################################################
        # TODO: Implement the prediction step for EKF                                 #
        # Hint: save your predicted state and cov as X_pred and P_pred                #
        ###############################################################################
        """
        print("")
        print("Debug prediction")
        print("")
        print("prediction X: ", X)
        print("prediction u: ", u)
        print("prediction P: ", P)
        print("prediction step: ", step)
        """
        X_pred = self.gfun(X,u)
        G = self.Gfun(X,u)
        #M = self.Q(u)
        M = self.M(u)
        #V = self.Vfun(X,u)
        #print("prediction V: ", V)
        #Covariance prediction
        #print("prediction self.M: ", self.M)
        #P_pred = G @ P @ G.T + V @ self.M @ V.T
        P_pred = G @ P @ G.T + M
        ###############################################################################
        #                         END OF YOUR CODE                                    #
        ###############################################################################

        self.state_.setState(X_pred)
        self.state_.setCovariance(P_pred)
        return np.copy(X_pred), np.copy(P_pred)


    def correction(self, z, landmarks, X, P):
        # EKF correction step
        #
        # Inputs:
        #   z:  measurement
        X_predict = X
        P_predict = P
        
        landmark1 = landmarks.getLandmark(z[2].astype(int))
        landmark2 = landmarks.getLandmark(z[5].astype(int))

        ###############################################################################
        # TODO: Implement the correction step for EKF                                 #
        # Hint: save your corrected state and cov as X and P                          #
        # Hint: you can use landmark1.getPosition()[0] to get the x position of 1st   #
        #       landmark, and landmark1.getPosition()[1] to get its y position        #
        ###############################################################################

        # Correction Step
        #z_hat = np.zeros(2)

        mu_pred1 = self.hfun(landmark1.getPosition()[0], landmark1.getPosition()[1], X_predict)
        mu_pred2 = self.hfun(landmark2.getPosition()[0], landmark2.getPosition()[1], X_predict)
        mu_pred1 = np.array(mu_pred1)
        mu_pred2 = np.array(mu_pred2)
        #print("mu_pred1:\n ", mu_pred1)
        #print("mu_pred2:\n ", mu_pred2)
        #print("mu_pred1.shape: ", mu_pred1.shape)
        #print("mu_pred2.shape: ", mu_pred2.shape)
        
        mu_pred1a = mu_pred1.reshape(2,1)
        mu_pred2a = mu_pred2.reshape(2,1)
        # Now stack them vertically
        mu_pred = np.vstack((mu_pred1a, mu_pred2a))

        #mu_pred = np.transpose(mu_pred)       #mu_pred = mu_pred.reshape(4, 1)
        print("mu_pred:\n ", mu_pred)
        print("mu_pred.shape: ", mu_pred.shape)
        # Step 1: Predicted mean: Already calculated via. prediction function.
        # Step 2: Predicted covariance: Already calculated via. prediction function.
        z_adjusted1 = np.array([[z[0], z[1]]])
        z_adjusted2 = np.array([[z[3], z[4]]])
        #print("z_adjusted1:\n ", z_adjusted1)
        #print("z_adjusted1.shape: ", z_adjusted1.shape)
        #print("z_adjusted2:\n ", z_adjusted2)
        #print("z_adjusted2.shape: ", z_adjusted2.shape)
        z_adjusted1 = z_adjusted1.reshape(2,1)
        z_adjusted2 = z_adjusted2.reshape(2,1)
        z_diff1 = z_adjusted1-mu_pred1a
        z_diff2 = z_adjusted2-mu_pred2a
        z_diff1[0] = wrap2Pi(z_diff1[0])
        z_diff2[0] = wrap2Pi(z_diff2[0])
        # Make it 4 rows, 1 column
        # Step 3: Calculate innovation.
        z_final = np.vstack((z_diff1,z_diff2))
        print("z_diff1:\n ", z_diff1)
        print("z_diff1.shape: ", z_diff1.shape)
        print("z_diff2:\n ", z_diff2)
        print("z_diff2.shape: ", z_diff2.shape)
        print("z_final:\n ", z_final)
        print("z_final.shape: ", z_final.shape) 
        

        # Above: for landmark2, try numbers 3 and 4, rather than 0 aand 1 respectively?
        H1 = self.Hfun(landmark1.getPosition()[0], landmark1.getPosition()[1], X_predict, mu_pred1)
        H2 = self.Hfun(landmark2.getPosition()[0], landmark2.getPosition()[1], X_predict, mu_pred2)
        #print("H1:\n ", H1)
        #print("H2:\n ", H2)
        #print("H1.shape: ", H1.shape)
        #print("H2.shape: ", H2.shape)
        H = np.vstack((H1,H2))
        #print("H:\n ", H)
        #print("H.shape: ", H.shape)
        # Step 4: Calculate innovation covariance.
        S = H @ P_predict @ H.T + block_diag(self.Q,self.Q)
        #print("S:\n ", S)
        #print("S.shape: ", S.shape)

        # Step 5: Filter Gain.
        K = P_predict @ H.T @ np.linalg.inv(S)

        #K2 = P_predict @ H2.T @ np.linalg.inv(S2)
        print("K:\n ", K)
        print("K.shape: ", K.shape)
        #print("X_predict pre reshape:\n", X_predict)

        #target_shape = (4,3)
        #padding = [(0, target_shape[0] - innovation.shape[0]), (0, target_shape[1] - innovation.shape[1])]
        # Adjust innovation to prevent dimensional mismatch.
        #innovation_padded = np.pad(innovation, padding, mode='constant')
        #print("innovation_padded:\n", innovation)
        X_predict = X_predict.reshape(3, 1)
        X = X_predict + (K @ z_final)
        print("X:\n", X)
        print("X.shape: ", X.shape)
        print("K @ innovation:\n ", K@z_final)
        print("K@innovation shape: ", (K@z_final).shape)
        #print("X_predict post reshape:\n", X_predict)
        #X[2] = wrap2Pi(X[2])
        #print("X post adjustment: ", X)
        P = P_predict - K @ S @ K.T 
        #P = P_predict - K @ S @ K.T
        #P2 = P_predict - K2 @ S2 @ K2.T
        print("P:\n", P)
        print("P.shape: ", P.shape)
        X = X.reshape(3,)
        print("X.shape: ", X.shape)
        #print("")
        #print("")

        ###############################################################################
        #                         END OF YOUR CODE                                    #
        ###############################################################################

        self.state_.setState(X)
        self.state_.setCovariance(P)
        return np.copy(X), np.copy(P)


    def getState(self):
        return deepcopy(self.state_)

    def setState(self, state):
        self.state_ = state