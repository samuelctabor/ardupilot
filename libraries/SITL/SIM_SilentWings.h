/*
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
/*
  simulator connection for ardupilot version of CRRCSim
*/

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a SilentWings simulator
 */
class SilentWings : public Aircraft {
public:
    SilentWings(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new SilentWings(home_str, frame_str);
    }

private:

    /*
      reply packet sent from SilentWings to ArduPilot
     */
    struct fdm_packet {
        uint16_t timestamp;          // Millisec  Timestamp
        double position_latitude;        // Degrees   Position latitude,
        double position_longitude;       // Degrees            longitude,
        float  altitude_ground;          // m         Altitude above gnd
        float  altitude_msl;             // m         Altitude - relative to Sea-level
        float  altitude_ground_45;       // m         gnd 45 degrees ahead (NOT IMPLEMENTED YET),

        float  roll;                     // Degrees
        float  pitch;                    // Degrees
        float  yaw;                      // Degrees

        float  d_roll;                    // Deg/sec   Roll speed.
        float  d_pitch;                   // Deg/sec   Pitch speed.
        float  d_yaw;                     // Deg/sec   Yaw speed.

        float  v_eas;                     // m/sec     Equivalent (indicated) air speed. 
        
        float  rate_of_turn;              // Deg/sec   Rate of turn.
        
        float  vz;                
        float  vx;                        // m/sec     Speed vector in body-axis
        float  vy;
        float  vz_wind; 
        float  vx_wind;                   // m/sec     Speed vector in body-axis, relative to wind
        float  vy_wind;
        
        float  ay;
        float  az;
        float  ax;                        // m/sec2    Acceleration vector in body axis

        float  angle_sideslip;           // Degrees   Sideslip angle
        float  vario;                     // m/sec     TE-compensated variometer.
        float  heading;                   // Degrees   Compass heading.
        float  angle_of_attack;          // Degrees   Angle of attack
        
        float  airpressure;               // pascal    Local air pressure (at aircraft altitude).
        float  density;                   // Air density at aircraft altitude.
        float  temperature;               // Celcius   Air temperature at aircraft altitude.
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);

    unsigned int last_timestamp;
    SocketAPM sock;

    Location loc1, loc2;

};

} // namespace SITL
