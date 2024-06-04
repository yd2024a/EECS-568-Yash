import numpy as np

# data from sensor I
z1 = [10.6715, 8.7925, 10.7172, 11.6302, 10.4889, 11.0347, 10.7269, 9.6966, 10.2939, 9.2127]

# data from sensor II
z2 = [10.7107, 9.0823, 9.1449, 9.3524, 10.2602]

# noise variance of sensor I
variance_z1 = 1

# noise variance of sensor II
variance_z2 = 0.64
#print("Debug A")

mean = 0
variance = 0
def Inference(mean, variance, variance_z, z):
    """
    MAP Bayesian inference using Gaussian prior and likelihood
    """
    #print("Debut B")
    #mean = 0
    #variance = 0
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    n = len(z)
    #mean = (variance/(variance+variance_z)) + mean* (variance_z/(variance + variance_z))
    print("mean: ", mean)
    print("variance: ", variance)
    if variance < 1e-6 or variance_z < 1e-6:
        raise ValueError("Variance values are way too small.")

    mean = (variance*np.mean(z)+variance_z*mean)/(variance+n*variance_z)
    #print("mean: ", mean)
    #variance = (variance * variance_z)/(variance + variance_z)
    variance = 1/(1/variance + n/variance_z)
    
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    return mean, variance
def Inference_func(dataset):
    mean_1 = np.mean(dataset)
    variance_1 = np.var(dataset)
    return mean_1, variance_1

def estimation_MAP():
    """
    recursive inference with data from sensor I and sensor II
    """
    # non-informative prior
    mean_1 = 0
    variance_1 = 1000
    mean_2 = 0
    variance_2 = 1000
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    
    # run inferece using z1 (use Inference_func instead of Inference)
    mean_1, variance_1 = Inference_func(z1)
    mean_2, variance_2 = Inference_func(z2)
    # print("DEBUG G: ", mean_1)
    # run inferece using z2 (use Inference_func instead of Inference)
    
    
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    return mean_1, variance_1, mean_2, variance_2

def KF_update(mean, variance, variance_R, z):
    """
    Kalman Filter Measurement Update
    """
    mean_c = 0
    variance_c = 0
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
   # H = 1
    # compute innovation statistics
    #v = z-H*mean
    #P = 0.64
   # R = 0.64
    #S = H*P*H.transpose()+R
    v = np.array(z) - mean
    S = variance + variance_R
    # Kalman gain
    K = variance/S
    
    # corrected mean and variance
    mean_c = mean + K * v
    variance_c = (1 -K)*variance

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    return mean_c, variance_c


def estimation_KF(KF_update_func):
    """
    recursive inference with data from sensor I and sensor II
    """
    # non-informative prior
    mean_1 = 0
    variance_1 = 1000
    mean_2 = 0
    variance_2 = 1000
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    
    # run inferece using z1 (use KF_update_func instead of KF_update)
    for measurement in z1:
        mean_1, variance_1 = KF_update_func(mean_1, variance_1, variance_z1, measurement)

    # run inferece using z2 (use KF_update_func instead of KF_update)
    for measurement in z2:
        mean_2, variance_2 = KF_update_func(mean_2, variance_2, variance_z2, measurement)
    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    return mean_1, variance_1, mean_2, variance_2

## BELOW: This was not originally there. I added it.

if __name__ == '__main__':
    # Test your funtions here
    test1 = Inference(np.mean(z2), variance_z1, variance_z2, z2)
    print('Test result: ', test1)
    test2a = Inference_func(z2)
    print('Inference result for sensor II:', test2a)  
    test2b = estimation_MAP()
    print('Test result: ', test2b)
    #test3 = KF_update()
    #print('Debug C')

   