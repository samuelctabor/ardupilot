#include "EKF_Thermal.h"
#include "AP_Math/matrixN.h"


float EKF_Thermal::measurementpredandjacobian(VectorN<float,N> &A, const VectorN<float,M> &U)
{
    // This function computes the Jacobian using equations from
    // analytical derivation of Gaussian updraft distribution
    // This expression gets used lots
    float expon = expf(- (powf(X[2]-U[2], 2) + powf(X[3]-U[3], 2)) / powf(X[1], 2));
    // Expected measurement
    float w = X[0] * expon;

    // Elements of the Jacobian
    A[0] = expon;
    A[1] = 2 * X[0] * ((powf(X[2]-U[2],2) + powf(X[3]-U[3],2)) / powf(X[1],3)) * expon;
    A[2] = -2 * (X[0] * (X[2]-U[2]) / powf(X[1],2)) * expon;
    A[3] = -2 * (X[0] * (X[3]-U[3]) / powf(X[1],2)) * expon;
    return w;
}

void EKF_Thermal::state_update(const VectorN<float,M> &U)
{
    // Estimate new state from old.
    X[2] += U[0];
    X[3] += U[1];
}

void EKF_Thermal::state_limits(void) 
{
    // Make sure values stay sensible.
    X[1] = X[1]>40.0 ? X[1]: 40.0;
}