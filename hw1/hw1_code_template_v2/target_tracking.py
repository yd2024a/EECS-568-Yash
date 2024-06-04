import numpy as np


def target_tracking():
    # known parameters
    mu0 = 20
    sigma0_square = 9
    F = 1
    Q = 4
    H = 1
    R = 1
    z1 = 22
    z2 = 23

    mu2 = 0
    sigma2_square = 0


    #Additional variables I am adding:
    #sigma_1_0_square = sigma0_square+Q
    meas_var = 1
    #K1 = sigma_1_0_square/(sigma_1_0_square + meas_var)



    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    # prediction step (1st iteration)
    mu1_hat = 20
    sigma1_square_hat = sigma0_square+Q

    # correction step (1st iteration)
    K1 = sigma1_square_hat/(sigma1_square_hat+meas_var)
    mu1 = mu1_hat + K1*(z1-mu1_hat)
    sigma1_square = (1-K1)*sigma1_square_hat

    # prediction step (2nd iteration)
    mu2_hat = mu1
    sigma2_square_hat = sigma1_square+Q

    # correction step (2nd iteration)
    K2 = (sigma2_square_hat)/(sigma2_square_hat+meas_var)
    mu2 = mu2_hat+K2*(z2-mu2_hat)
    sigma2_square = (1-K2)*sigma2_square_hat

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################

    return (mu2, sigma2_square)


if __name__ == '__main__':
    # Test your funtions here

    print('Answer for Problem 3:\n', target_tracking())
