/*
    Extended Kalman Filter class by Sam Tabor, 2013.
    * http://diydrones.com/forum/topics/autonomous-soaring
    * Set up for identifying thermals of Gaussian form, but could be adapted to other
    * purposes by adapting the equations for the jacobians.

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
#include "ExtendedKalmanFilter.h"

class EKF_Thermal : public ExtendedKalmanFilter<4,4> {

    static constexpr const uint8_t N = 4;
    static constexpr const uint8_t M = 4;

private:
    float measurementpredandjacobian(VectorN<float,N> &A, const VectorN<float,M> &U) override;

    void state_update(const VectorN<float,M> &U) override;

    void state_limits(void) override;
};
