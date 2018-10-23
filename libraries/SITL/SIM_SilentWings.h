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
    static const unsigned int PKT_LEN = 132; 

    /*
      reply packet sent from SilentWings to ArduPilot
     */
    struct PACKED fdm_packet {
           unsigned int timestamp;          // Millisec  Timestamp
           double position_latitude;        // Degrees   Position latitude,
           double position_longitude;       // Degrees            longitude,
           float  altitude_msl;             // m         Altitude - relative to Sea-level
           float  altitude_ground;          // m         Altitude above gnd
           float  altitude_ground_45;       // m         gnd 45 degrees ahead (NOT IMPLEMENTED YET),
           float  altitude_ground_forward;  // m         gnd straight ahead (NOT IMPLEMENTED YET).
           float  roll;                     // Degrees
           float  pitch;                    // Degrees
           float  yaw;                      // Degrees
           float  d_roll;                    // Deg/sec   Roll speed.
           float  d_pitch;                   // Deg/sec   Pitch speed.
           float  d_yaw;                     // Deg/sec   Yaw speed.
           float  vx;                        // m/sec     Speed vector in body-axis
           float  vy; 
           float  vz;                
           float  vx_wind;                   // m/sec     Speed vector in body-axis, relative to wind
           float  vy_wind;
           float  vz_wind; 
           float  v_eas;                     // m/sec     Equivalent (indicated) air speed. 
           float  ax;                        // m/sec2    Acceleration vector in body axis
           float  ay;
           float  az;
           float  angle_of_attack;          // Degrees   Angle of attack
           float  angle_sideslip;           // Degrees   Sideslip angle
           float  vario;                     // m/sec     TE-compensated variometer.
           float  heading;                   // Degrees   Compass heading.
           float  rate_of_turn;              // Deg/sec   Rate of turn.
           float  airpressure;               // pascal    Local air pressure (at aircraft altitude).
           float  density;                   // Air density at aircraft altitude.
           float  temperature;               // Celcius   Air temperature at aircraft altitude.
    };

    enum data_byte_offset {
        TIMESTAMP_OFFSET = 0,
        LAT_OFFSET = TIMESTAMP_OFFSET + sizeof(fdm_packet::timestamp),
        LON_OFFSET = LAT_OFFSET + sizeof(fdm_packet::position_latitude),
        ALT_MSL_OFFSET = LON_OFFSET + sizeof(fdm_packet::position_longitude),
        ALT_GND_OFFSET = ALT_MSL_OFFSET + sizeof(fdm_packet::altitude_msl),
        ALT_GND_45_OFFSET = ALT_GND_OFFSET + sizeof(fdm_packet::altitude_ground),
        ALT_GND_FWD_OFFSET = ALT_GND_45_OFFSET + sizeof(fdm_packet::altitude_ground_45),
        ROLL_OFFSET = ALT_GND_FWD_OFFSET + sizeof(fdm_packet::altitude_ground_forward),
        PITCH_OFFSET = ROLL_OFFSET + sizeof(fdm_packet::roll),
        YAW_OFFSET = PITCH_OFFSET + sizeof(fdm_packet::pitch),
        D_ROLL_OFFSET = YAW_OFFSET + sizeof(fdm_packet::yaw),
        D_PITCH_OFFSET = D_ROLL_OFFSET + sizeof(fdm_packet::d_roll),
        D_YAW_OFFSET = D_PITCH_OFFSET + sizeof(fdm_packet::d_pitch),
        VX_OFFSET = D_YAW_OFFSET + sizeof(fdm_packet::d_yaw),
        VY_OFFSET = VX_OFFSET + sizeof(fdm_packet::vx),
        VZ_OFFSET = VY_OFFSET + sizeof(fdm_packet::vy),
        VX_WIND_OFFSET = VZ_OFFSET + sizeof(fdm_packet::vz),
        VY_WIND_OFFSET = VX_WIND_OFFSET + sizeof(fdm_packet::vx_wind),
        VZ_WIND_OFFSET = VY_WIND_OFFSET + sizeof(fdm_packet::vy_wind),
        V_EAS_OFFSET = VZ_WIND_OFFSET + sizeof(fdm_packet::vz_wind),
        AX_OFFSET = V_EAS_OFFSET + sizeof(fdm_packet::v_eas),
        AY_OFFSET = AX_OFFSET + sizeof(fdm_packet::ax),
        AZ_OFFSET = AY_OFFSET + sizeof(fdm_packet::ay),
        AOA_OFFSET = AZ_OFFSET + sizeof(fdm_packet::az),
        AS_OFFSET  = AOA_OFFSET + sizeof(fdm_packet::angle_of_attack),
        VARIO_OFFSET = AS_OFFSET + sizeof(fdm_packet::angle_sideslip),
        HEADING_OFFSET = VARIO_OFFSET + sizeof(fdm_packet::vario),
        ROT_OFFSET = HEADING_OFFSET + sizeof(fdm_packet::heading),
        AIRPRESSURE_OFFSET = ROT_OFFSET + sizeof(fdm_packet::rate_of_turn),
        DENSITY_OFFSET = AIRPRESSURE_OFFSET + sizeof(fdm_packet::airpressure),
        TEMP_OFFSET = DENSITY_OFFSET + sizeof(fdm_packet::density)
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);

    unsigned int last_timestamp;
    SocketAPM sock;

    Location curr_location;
    bool home_initialized;
};

} // namespace SITL
