#include "ExtendedKalmanFilter.h"
#include "AP_Math/matrixN.h"

template <uint8_t N, uint8_t M>
void ExtendedKalmanFilter<N,M>::reset(const VectorN<float,N> &x, const MatrixN<float,N> &p, const MatrixN<float,N> q, float r)
{
    P = p;
    X = x;
    Q = q;
    R = r;
}

template <uint8_t N, uint8_t M>
void ExtendedKalmanFilter<N,M>::update(float z, const VectorN<float,M> &U)
{
    MatrixN<float,N> tempM;
    VectorN<float,N> H;
    VectorN<float,N> P12;
    VectorN<float,N> K;
    
    // U: [driftX,driftY,Px,Py]

    // LINE 28
    state_update(U);

    // LINE 33
    // Update the covariance matrix
    // P = A*ekf.P*A'+ekf.Q;
    // We know A is identity so
    // P = ekf.P+ekf.Q;
    P += Q;

    // What measurement do we expect to receive in the estimated
    // state
    // LINE 37
    // [z1,H] = ekf.jacobian_h(x1);
    float z1 = measurementpredandjacobian(H, U);

    // LINE 40
    // P12 = P * H';
    P12.mult(P, H); //cross covariance 
    
    // LINE 41
    // Calculate the KALMAN GAIN
    // K = P12 * inv(H*P12 + ekf.R);                     %Kalman filter gain
    K = P12 * 1.0 / (H * P12 + R);

    // Correct the state estimate using the measurement residual.
    // LINE 44
    // X = x1 + K * (z - z1);
    X += K * (z - z1);

    // Make sure values stay sensible.
    state_limits();

    // Correct the covariance too.
    // LINE 46
    // NB should be altered to reflect Stengel
    // P = P_predict - K * P12';
    tempM.mult(K, P12);
    P -= tempM;
    
    P.force_symmetry();
}

template void ExtendedKalmanFilter<4,4>::reset(const VectorN<float,4> &x, const MatrixN<float,4> &p, const MatrixN<float,4> q, float r);
template void ExtendedKalmanFilter<4,4>::update(float z, const VectorN<float,4> &U);

template void ExtendedKalmanFilter<2,3>::reset(const VectorN<float,2> &x, const MatrixN<float,2> &p, const MatrixN<float,2> q, float r);
template void ExtendedKalmanFilter<2,3>::update(float z, const VectorN<float,3> &U);

