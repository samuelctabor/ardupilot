/*
   EKF base class for estimators with single measurement and trivial state
   dynamics. This could be generalised to more complex state dynamics.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <AP_Math/matrixN.h>

template <uint8_t N, uint8_t M>
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(void) {}

    VectorN<float,N> X;
    MatrixN<float,N> P;
    MatrixN<float,N> Q;
    float R;
    void reset(const VectorN<float,N> &x, const MatrixN<float,N> &p, const MatrixN<float,N> q, float r);
    void update(float z, const VectorN<float,M> &U);

private:
    virtual float measurementpredandjacobian(VectorN<float,N> &A, const VectorN<float,M> &U);

    virtual void state_update(const VectorN<float,M> &U);

    virtual void state_limits(void);
};
