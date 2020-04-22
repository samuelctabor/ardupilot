#include "EKF_Polar.h"
#include "AP_Math/matrixN.h"


float EKF_Polar::measurementpredandjacobian(VectorN<float,N> &A, const VectorN<float,M> &U)
{
    // This function calculates the Jacobian using equations of theoretical sinkrate as
    // a function of the glide polar parameters.
    // Vz = CD0 * Vx^3 / K + B*K/Vx, where K = (2*m*g)/(1.225*S)

    float V = U[0];
    // float roll = U[1];
    float k = U[2];

    A[0] = powf(V,3) / k;
    A[1] = k / V;

    // Expected measurement
    float w = X[0] * A[0] + X[1] * A[1];
    
    return w;
}

void EKF_Polar::state_update(const VectorN<float,M> &U)
{
    // Estimate new state from old.
}

void EKF_Polar::state_limits(void)
{
    // Make sure values stay sensible.
    X[0] = X[0]>0.02 ? X[0]: 0.02;
    X[1] = X[1]>0.02 ? X[1]: 0.02;
    X[0] = X[0]<0.10 ? X[0]: 0.10;
    X[1] = X[1]<0.10 ? X[1]: 0.10;
}