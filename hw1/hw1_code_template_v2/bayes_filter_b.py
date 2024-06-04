import numpy as np
import matplotlib.pyplot as plt

# colors
green = np.array([0.2980, 0.6, 0])
darkblue = np.array([0, 0.2, 0.4])
VermillionRed = np.array([156, 31, 46]) / 255


def plot_fuction(belief, prediction, posterior_belief):
    """
    plot prior belief, prediction after action, and posterior belief after measurement
    """
    fig = plt.figure()

    # plot prior belief
    ax1 = plt.subplot(311)
    plt.bar(np.arange(0, 10), belief.reshape(-1), color=darkblue)
    plt.title(r'Prior Belief')
    plt.ylim(0, 1)
    plt.ylabel(r'$bel(x_{t-1})$')

    # plot likelihood
    ax2 = plt.subplot(312)
    plt.bar(np.arange(0, 10), prediction.reshape(-1), color=green)
    plt.title(r'Prediction After Action')
    plt.ylim(0, 1)
    plt.ylabel(r'$\overline{bel(x_t})}$')

    # plot posterior belief
    ax3 = plt.subplot(313)
    plt.bar(np.arange(0, 10), posterior_belief.reshape(-1), color=VermillionRed)
    plt.title(r'Posterior Belief After Measurement')
    plt.ylim(0, 1)
    plt.ylabel(r'$bel(x_t})$')

    plt.show()


def bayes_filter_b():
    """
    Follow steps of Bayes filter.  
    You can use the plot_fuction() above to help you check the belief in each step.
    Please print out the final answer.
    """

    # Initialize belief uniformly
    belief = 0.1 * np.ones(10)

    posterior_belief = np.zeros(10)
    #############################################################################
    #                    TODO: Implement you code here                          #
    #############################################################################
    # Measurement model
    # Measurement model returns p(z|x) based on a likelihood map
    def measurement_model(x, likelihood_map):
        return likelihood_map[:, x]
    # Control Inputs (Sequence of Actions)
    u = [0,3,4]
    # Measurement
    z = [1,1,0]
    # State Space: The world has 10 places. After 9, the robot can stay, go 3 steps CCW (at 2), or 4 steps CCW (at 3).
    X = np.arange(0,10,1)

    def motion_model(xi,xj,u):
        if u ==3: #CCW by 3
            dx = xi-xj
            if dx == -7 or dx == 3:
                p = 1
            else:
                p = 0
        elif u == 4: #CCW by 4
            dx = xi-xj
            if dx == -6 or dx == 4:
                p = 1
            else:
                p = 0
        elif u == 0: #Stay
            dx = xi-xj
            if dx == 0:
                p = 1
            else:
                p = 0
        return p
          

    # Likelihood map to provide measurements.
    # Measurements should be unique at points 0, 6, and 9.
    likelihood_map = np.ones([1, 10]) * 0.4
    for i in [0, 6, 9]:
        likelihood_map[:, i] = 0.8
    # Markov Localization using Bayes filter (Not sure if Markov localization is needed, but Bayes filter is).
    # This main loop can be run forever, but we run it for a limited sequence of control inputs
    #fig2 = plt.figure()
    
    k = 0 # step counter
    while len(u):
        if z[k] == 1: # measurement received
            eta = 0 # normalization constant
            for i in range(len(X)):
                likelihood = measurement_model(X[i], likelihood_map) #get measurement likelihood
                belief[i] = likelihood[i]*posterior_belief[i] #unnormalized Bayes update, LIKELIHOOD IS AN ARRAY. DATA TYPE MISMATCH?
                eta = eta + belief[i] # normalize belief
            #Print OUT ETA....
            print("Testing A: ", eta)
            belief = belief/eta #normalize belief

    #prediction; belief convolution
    #for m in range(len(X)):
    #    posterior_belief[m] = 0
    #    for n in range(len(X)):
    #        pu = motion_model(X[m], X[n], u[0])
    #        posterior_belief[m] = posterior_belief[m] + pu + belief[n]

    # set the predicted belief as prior
    #belief = np.copy(posterior_belief)

    # remove the executed action from the list
    u.remove(u[0])
    k = k+1
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    return posterior_belief


if __name__ == '__main__':
    # Test your funtions here
    belief = bayes_filter_b()
    print('Answer for Problem 2b:')
    for i in range(10):
        print("%6d %18.3f\n" % (i, belief[i]))
    plt.bar(np.arange(0, 10), belief)
