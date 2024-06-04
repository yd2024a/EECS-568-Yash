import numpy as np
import matplotlib.pyplot as plt

def calculateEllipseXY(mu, Sigma, k2, N=20):
    """
    input:
    mu     is the [2x1] [x;y] mean vector
    Sigma  is the [2x2] covariance matrix
    k2     is the Chi-squared 2 DOF variable
    N      is the number of points to use (optional, default 20)
    
    output:
    x and y coordinates to draw the ellipse
    """
    # set up angles for drawing the ellipse
    angles = np.linspace(0, 2*np.pi, num=N)
    _circle = np.array([np.cos(angles), np.sin(angles)])

    # make sure it is a numpy array
    mu = np.array(mu)
    Sigma = np.array(Sigma)
        
    # cholesky decomposition
    L = np.linalg.cholesky(Sigma) # Cholesky factor of covariance
    
    # apply the transformation and scale of the covariance
    ellipse = np.sqrt(k2) * L @ _circle

    # shift origin to the mean
    x = mu[0] + ellipse[0, :].T
    y = mu[1] + ellipse[1, :].T

    return x, y

def draw_ellipse(mu, Sigma, k2, colorin='red'):
    """   
    input:
    mu       is the [2x1] [x;y] mean vector
    Sigma    is the [2x2] covariance matrix
    k2       is the Chi-squared 2 DOF variable
    Npoints  number of points to draw the ellipse (default 20)
    colorin  color for plotting ellipses, red for analytical contours, blue for sample contours
    
    --- h = draw_ellipse(mu, Sigma, k2)
    Draws an ellipse centered at mu with covariance Sigma and confidence region k2, i.e.,
    K2 = 1; # 1-sigma
    K2 = 4; # 2-sigma
    K2 = 9; # 3-sigma
    K2 = chi2inv(.50, 2); # 50% probability contour
    """
    Npoints = 20
    
    x, y = calculateEllipseXY(mu, Sigma, k2, Npoints)
    
    if k2 == 9:
        if colorin == 'red':
            plt.plot(x, y, linewidth=1.25, color=colorin, label='analytical contours')
        elif colorin == 'blue':
            plt.plot(x, y, linewidth=1.25, color=colorin, label='sample contours')
    else:
        plt.plot(x, y, linewidth=1.25, color=colorin)

def covariance_propagation():
    # parameter setting
    N = 10000
    mu_sensor = [10, 0]
    sigma_sensor = [0.5, 0.25]

    # output
    result = {'N': N, 'mu_sensor': mu_sensor, 'sigma_sensor': sigma_sensor}

    #############################################################################
    #                                 Problem 4a                                #
    #############################################################################

    # generate point clouds
    r, theta = np.zeros(10), np.zeros(10)
    x, y = np.zeros(10), np.zeros(10)
    random_values1 = np.random.randn(N)
    random_values2 = np.random.randn(N)
    #print('Testing: ', random_values1)
    #print('Testing, part 2: ', random_values2)
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    # i) Sensor (r, theta) frame
    # note: use random_values1 and random_values2 here due to autograder requirement
    
    r = mu_sensor[0]+sigma_sensor[0]*random_values1
    theta = mu_sensor[1]+sigma_sensor[1]*random_values2

    # ii) Cartesian (x,y) coordinate frame
    
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    result['r'] = r
    result['theta'] = theta
    result['x'] = x
    result['y'] = y

    #############################################################################
    #                                 Problem 4b                                #
    #############################################################################
    Jacobian = np.zeros((2,2))
    cov_cartesian = np.zeros((2,2))
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    print("")
    # Implement the Jacobians
    #Jacobian = [[np.cos(mu_sensor[1]), -r*np.sin(mu_sensor[1])],[np.sin(mu_sensor[1]), r*np.cos(mu_sensor[1])]]
    
    # Implement the linearized covariance in cartesian corridinates
    #sigma_x_squared = ((np.cos(mu_sensor[1]))**2)*(sigma_sensor[0])**2 + ((r*np.sin(mu_sensor[1]))**2)*(sigma_sensor[1])**2
    #sigma_y_squared = ((np.sin(mu_sensor[1]))**2)*(sigma_sensor[0])**2 + ((r*np.cos(mu_sensor[1]))**2)*(sigma_sensor[1])**2
    #cov_cartesian = [[sigma_x_squared, 0],[0, sigma_y_squared]]
    # Implement the Jacobians
    Jacobian = np.array([[np.cos(mu_sensor[1]), -mu_sensor[0] * np.sin(mu_sensor[1])],
                        [np.sin(mu_sensor[1]), mu_sensor[0] * np.cos(mu_sensor[1])]])

    # Implement the linearized covariance in Cartesian coordinates
    sigma_x_squared = (np.cos(mu_sensor[1])**2) * (sigma_sensor[0]**2) + (mu_sensor[0] * np.sin(mu_sensor[1]))**2 * (sigma_sensor[1]**2)
    sigma_y_squared = (np.sin(mu_sensor[1])**2) * (sigma_sensor[0]**2) + (mu_sensor[0] * np.cos(mu_sensor[1]))**2 * (sigma_sensor[1]**2)

    cov_cartesian = np.array([[sigma_x_squared, 0], [0, sigma_y_squared]])
    print("sigma_x_squared: ", sigma_x_squared)
    print("sigma_y_squared: ", sigma_y_squared)
    print("Jacobian: ", Jacobian)
    print("cov_cartesian: ",cov_cartesian)
    print("r: ", r)
    print("mu_sensor[1]: ", mu_sensor[1])
    print("Dimensions of r:", r.shape)
    print("theta: ", theta)
    print("Dimensions of theta: ", theta.shape)
    print("")
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    result['Jacobian'] = Jacobian
    result['cov_cartesian'] = cov_cartesian


    #############################################################################
    #                                 Problem 4c                                #
    #############################################################################
    
    # Sensor frame
    # compute the mean and covariance of samples in the sensor frame 
    # mu_sensor is given
    cov_sensor = np.zeros((2,2))
    mu_sensor_sample = np.zeros((2,))
    cov_sensor_sample = np.zeros((2,2))
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    cov_sensor = [[0.5**2, 0],[0, 0.25**2]]
    mu_sensor_sample = np.array([np.mean(r), np.mean(theta)])
    cov_sensor_sample = np.cov([r,theta])
    print("Debug C: ", mu_sensor_sample)
    print("Debug D: ", cov_sensor_sample)

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    result['mu_sensor'] = mu_sensor
    result['cov_sensor'] = cov_sensor
    result['mu_sensor_sample'] = mu_sensor_sample
    result['cov_sensor_sample'] = cov_sensor_sample

    # Cartesian frame
    # compute the mean and covariance of samples in the Cartesian frame
    mu_cartesian = np.zeros((2,))
    # cov_cartesian is calculated in Problem 4b
    mu_cartesian_sample = np.zeros((2,))
    cov_cartesian_sample = np.zeros((2,2))
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    mu_cartesian = Jacobian @ mu_sensor
    mu_cartesian_sample = Jacobian @ mu_sensor_sample
    cov_cartesian_sample = Jacobian @ cov_sensor_sample
    print("Debug E: ", mu_cartesian)
    print("Debug F: ", mu_cartesian_sample)
    print("Debug G: ", cov_cartesian_sample)
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    result['mu_cartesian'] = mu_cartesian
    result['cov_cartesian'] = cov_cartesian
    result['mu_cartesian_sample'] = mu_cartesian_sample
    result['cov_cartesian_sample'] = cov_cartesian_sample

    print('Debug: Answer for Problem 4c:')

    #############################################################################
    #                                 Problem 4d                                #
    #############################################################################
    # counter for samples lie within the contour
    count_sensor = np.zeros((3,1)) # count results of samples in the sensor frame
    count_cartesian = np.zeros((3,1)) # count results of samples in the Cartesian frame

    # Compute the Mahalabobis distance of samples, and count how many samples lie in the contour
    #############################################################################
    #                    TODO: Implement your code here                         #
    
    #############################################################################
    print("Debug Z: ", count_sensor)
    print("Debug Y: ", count_cartesian)
    cov_sensor = np.array(cov_sensor)
    cov_cartesian = np.array(cov_cartesian)
    print("Debug X: ", cov_sensor)
    print("Debug W: ", cov_cartesian)   

    t = [1, 4, 9]

        # Compute Mahalanobis distance and sample counting.
    for i, threshold in enumerate(t):
        md_sensor = ((r - mu_sensor[0])**2 / cov_sensor[0, 0]) + ((theta - mu_sensor[1])**2 / cov_sensor[1, 1])
        count_sensor[i] = np.sum(md_sensor < threshold)

        # Compute Mahalanobis distance and sample counting for cartesion frame.
    for i, threshold in enumerate(t):
        md_cartesian = ((x - mu_cartesian[0])**2 / cov_cartesian[0, 0]) + ((y - mu_cartesian[1])**2 / cov_cartesian[1, 1])
        count_cartesian[i] = np.sum(md_cartesian < threshold)


    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    result['count_sensor'] = count_sensor
    result['count_cartesian'] = count_cartesian


    #############################################################################
    #                                 Problem 4e                                #
    #############################################################################
    result['4e'] = {}
    for rho in [0.1, 0.5, 0.9]: # correlation coefficient, eg. 0.1, 0.5, 0.9
        result['4e'][rho] = {}
        
        # Part A. generate point clouds
        mu_sensor = np.zeros((2,))
        cov_sensor = np.zeros((2,2))
        r, theta = np.zeros(10), np.zeros(10)
        x, y = np.zeros(10), np.zeros(10)
        #############################################################################
        #                    TODO: Implement your code here                         #
        #############################################################################
        # i) Sensor (r, theta) frame 
        
        # mu_sensor = ...
        # cov_sensor = ...
        
        # note: do not change the next line due to autograder requirement
        r, theta = np.random.multivariate_normal(mu_sensor, cov_sensor, N).T

        # ii) Cartesian (x,y) coordinate frame
        # x = ...
        # y = ...

        #############################################################################
        #                            END OF YOUR CODE                               #
        #############################################################################
        result['4e'][rho]['mu_sensor'] = mu_sensor
        result['4e'][rho]['cov_sensor'] = cov_sensor
        result['4e'][rho]['r'] = r
        result['4e'][rho]['theta'] = theta
        result['4e'][rho]['x'] = x
        result['4e'][rho]['y'] = y

        # Part C. Draw ellipse
        # compute the mean and covariance of samples in the sensor frame 
        mu_sensor_sample = np.zeros((2,))
        cov_sensor_sample = np.zeros((2,2))
        #############################################################################
        #                    TODO: Implement your code here                         #
        #############################################################################

        # mu_sensor_sample = ...
        # cov_sensor_sample = ...

        #############################################################################
        #                            END OF YOUR CODE                               #
        #############################################################################
        result['4e'][rho]['mu_sensor_sample'] = mu_sensor_sample
        result['4e'][rho]['cov_sensor_sample'] = cov_sensor_sample

        # compute the mean and covariance of samples in the Cartesian frame
        mu_cartesian = np.zeros((2,))
        cov_cartesian = np.zeros((2,2))
        mu_cartesian_sample = np.zeros((2,))
        cov_cartesian_sample = np.zeros((2,2))
        #############################################################################
        #                    TODO: Implement your code here                         #
        #############################################################################

        # mu_cartesian = ...
        # cov_cartesian = ...
        # mu_cartesian_sample = ...
        # cov_cartesian_sample = ...

        #############################################################################
        #                            END OF YOUR CODE                               #
        #############################################################################
        result['4e'][rho]['mu_cartesian'] = mu_cartesian
        result['4e'][rho]['cov_cartesian'] = cov_cartesian
        result['4e'][rho]['mu_cartesian_sample'] = mu_cartesian_sample
        result['4e'][rho]['cov_cartesian_sample'] = cov_cartesian_sample

    return result

if __name__ == '__main__':
    # Test your funtions here
    result = covariance_propagation()
    print('Answer for Problem 4d:\n', result)