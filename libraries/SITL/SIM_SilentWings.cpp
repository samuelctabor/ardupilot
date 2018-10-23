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
  simulator connector for ardupilot version of SilentWings
*/

#include "SIM_SilentWings.h"

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

SilentWings::SilentWings(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    last_timestamp(0),
    sock(true),
    home_initialized(false)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // SilentWings keeps sending us packets. Not strictly necessary but
    // useful for debugging
    sock.bind("127.0.0.1", 6060);

    sock.reuseaddress();
    sock.set_blocking(false);
}

/*
  decode and send servos
*/
void SilentWings::send_servos(const struct sitl_input &input)
{
    /*
    char *buf = nullptr;
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float throttle = filtered_servo_range(input, 2);
    float rudder   = filtered_servo_angle(input, 3);
    float flap     = 0.0f;
  
    float wind_speed_fps = input.wind.speed / FEET_TO_METERS;
    asprintf(&buf,
             "AIL %f\n"
             "ELE %f\n"
             "RUD %f\n"
             "THR %f\n"
             "FLP %f\n",
             aileron, elevator, rudder, throttle,flap);

    ssize_t buflen = strlen(buf);
    
    ssize_t sent = sock.sendto(buf, buflen, "127.0.0.1", 6070);
    free(buf);

    if (sent < 0) {
        fprintf(stderr, "Fatal: Failed to send on control socket\n"),
        exit(1);
    }
    
    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }
    */

}

/*
  receive an update from the FDM
  This is a blocking function
 */
void SilentWings::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;
    memset(&pkt, 0, sizeof(pkt));
    const unsigned int *data_ui = nullptr;
    const float *data_f = nullptr;
    const double *data_d = nullptr;
    uint8_t raw_pkt[PKT_LEN];
    
    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    //while (sock.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
    //uint8_t nread = sock.recv(&pkt_tmp, sizeof(pkt_tmp), 0);
    
    uint8_t nread = sock.recv(&raw_pkt, PKT_LEN, 0);
    
    if (nread != PKT_LEN) {
        return;
    }
    
    // printf("Received %i bytes\n", nread); 

    data_ui = (const unsigned int*)&raw_pkt[TIMESTAMP_OFFSET];
    pkt.timestamp = *data_ui;
    
    data_d = (const double*)&raw_pkt[LAT_OFFSET];
    pkt.position_latitude = *data_d;
    
    data_d = (const double*)&raw_pkt[LON_OFFSET];
    pkt.position_longitude = *data_d;
    
    data_f = (const float*)&raw_pkt[ALT_MSL_OFFSET];
    pkt.altitude_msl = *data_f;
    
    data_f = (const float*)&raw_pkt[ALT_GND_OFFSET];
    pkt.altitude_ground = *data_f;
    
    data_f = (const float*)&raw_pkt[ALT_GND_45_OFFSET];
    pkt.altitude_ground_45 = *data_f;
    
    data_f = (const float*)&raw_pkt[ALT_GND_FWD_OFFSET];
    pkt.altitude_ground_forward = *data_f;
    
    data_f = (const float*)&raw_pkt[ROLL_OFFSET];
    pkt.roll = *data_f;
    
    data_f = (const float*)&raw_pkt[PITCH_OFFSET];
    pkt.pitch = *data_f;
    
    data_f = (const float*)&raw_pkt[YAW_OFFSET];
    pkt.yaw = *data_f;
    
    data_f = (const float*)&raw_pkt[D_ROLL_OFFSET];
    pkt.d_roll = *data_f;
    
    data_f = (const float*)&raw_pkt[D_PITCH_OFFSET];
    pkt.d_pitch = *data_f;
    
    data_f = (const float*)&raw_pkt[D_YAW_OFFSET];
    pkt.d_yaw = *data_f;
    
    data_f = (const float*)&raw_pkt[VX_OFFSET];
    pkt.vx = *data_f;
    
    data_f = (const float*)&raw_pkt[VY_OFFSET];
    pkt.vy = *data_f;
    
    data_f = (const float*)&raw_pkt[VZ_OFFSET];
    pkt.vz = *data_f;
    
    data_f = (const float*)&raw_pkt[VX_WIND_OFFSET];
    pkt.vx_wind = *data_f;
    
    data_f = (const float*)&raw_pkt[VY_WIND_OFFSET];
    pkt.vy_wind = *data_f;
    
    data_f = (const float*)&raw_pkt[VZ_WIND_OFFSET];
    pkt.vz_wind = *data_f;
    
    data_f = (const float*)&raw_pkt[V_EAS_OFFSET];
    pkt.v_eas = *data_f;
    
    data_f = (const float*)&raw_pkt[AX_OFFSET];
    pkt.ax = *data_f;
    
    data_f = (const float*)&raw_pkt[AY_OFFSET];
    pkt.ay = *data_f;
    
    data_f = (const float*)&raw_pkt[AZ_OFFSET];
    pkt.az = *data_f;
    
    data_f = (const float*)&raw_pkt[AOA_OFFSET];
    pkt.angle_of_attack = *data_f;
    
    data_f = (const float*)&raw_pkt[AS_OFFSET];
    pkt.angle_sideslip = *data_f;
    
    data_f = (const float*)&raw_pkt[VARIO_OFFSET];
    pkt.vario = *data_f;
    
    data_f = (const float*)&raw_pkt[HEADING_OFFSET];
    pkt.heading = *data_f;
    
    data_f = (const float*)&raw_pkt[ROT_OFFSET];
    pkt.rate_of_turn = *data_f;
    
    data_f = (const float*)&raw_pkt[AIRPRESSURE_OFFSET];
    pkt.airpressure = *data_f;
    
    data_f = (const float*)&raw_pkt[DENSITY_OFFSET];
    pkt.density = *data_f;
    
    data_f = (const float*)&raw_pkt[TEMP_OFFSET];
    pkt.temperature = *data_f;
    
    // auto-adjust to crrcsim frame rate
    // QUESION: Do we actually need this?
    double deltat = (pkt.timestamp - last_timestamp)/1000.0f;

    if (deltat > 100000.0) {
        return;
    }

    /*
    printf("------------\ntimestamp: %d\nposition_latitude: %f\nposition_longitude: %f\naltitude_ground: %f\naltitude_msl: %f\naltitude_ground_45: %f\nroll: %f\npitch: %f\nyaw: %f\nd_roll: %f\nd_pitch %f\nd_yaw %f\nv_eas%f\nrate_of_turn %f\nvz %f\nvx %f\nvy %f\nvz_wind %f\nvx_wind %f\nvy_wind %f\nay %f\naz %f\nax %f\nangle_sideslip %f\nvario %f\nheading %f\nangle_of_attack %f\nairpressure %f\ndensity %f\ntemperature %f\n-----------\n", 
        pkt.timestamp, 
        pkt.position_latitude, 
        pkt.position_longitude,
        pkt.altitude_ground,
        pkt.altitude_msl,
        pkt.altitude_ground_45,
        pkt.roll,
        pkt.pitch,
        pkt.yaw,
        pkt.d_roll,
        pkt.d_pitch,
        pkt.d_yaw,
        pkt.v_eas,
        pkt.rate_of_turn,
        pkt.vz,
        pkt.vx,
        pkt.vy,
        pkt.vz_wind,
        pkt.vx_wind,
        pkt.vy_wind,
        pkt.ay,
        pkt.az,
        pkt.ax,
        pkt.angle_sideslip,
        pkt.vario,
        pkt.heading,
        pkt.angle_of_attack,
        pkt.airpressure,
        pkt.density,
        pkt.temperature
        );
    */

    dcm.from_euler(radians(pkt.roll), radians(pkt.pitch), radians(pkt.yaw));    
    // This is g-load.
    accel_body = Vector3f(pkt.ax * GRAVITY_MSS, pkt.ay * GRAVITY_MSS, pkt.az * GRAVITY_MSS); 
    gyro = Vector3f(radians(pkt.d_roll), radians(pkt.d_pitch), radians(pkt.d_yaw));
    velocity_ef = Vector3f(pkt.vx, pkt.vy, pkt.vz);
    wind_ef = velocity_ef - Vector3f(pkt.vx_wind, pkt.vy_wind, pkt.vz_wind);
    airspeed = pkt.v_eas;
    airspeed_pitot = pkt.v_eas;
    curr_location.lat = pkt.position_latitude * 1.0e7;
    curr_location.lng = pkt.position_longitude * 1.0e7;
    curr_location.alt = pkt.altitude_msl * 100.0f;
    ground_level = curr_location.alt * 0.01f - pkt.altitude_ground;
    Vector3f posdelta = location_3d_diff_NED(home, curr_location);
    position.x = posdelta.x;
    position.y = posdelta.y;
    position.z = posdelta.z;
    
    update_position();
    time_advance();

    if (get_distance(curr_location, location) > 4 || abs(curr_location.alt - location.alt)*0.01f > 2.0f || !home_initialized) {
        printf("SilentWings home reset dist=%f alt=%.1f/%.1f\n",
               get_distance(curr_location, location), curr_location.alt*0.01f, location.alt*0.01f);
        // reset home location
        home.lat = curr_location.lat;
        home.lng = curr_location.lng;
        home.alt = curr_location.alt;
        position.x = 0;
        position.y = 0;
        position.z = 0;
        home_initialized = true;
        update_position();
        time_advance();
    }
    
    // printf("Ground level: %f; pos-x: %f; pos-y: %f; pos-z: %f; location-z(alt) in meters: %f; curr_location-z(alt) in meters: %f; alt_msl: %f; alt_ground: %f\n", ground_level, position.x, position.y, position.z, location.alt*0.01f, curr_location.alt*0.01f, pkt.altitude_msl, pkt.altitude_ground);

    time_now_us += deltat * 1.0e6;

    // printf("Delta: %f; Time: %d; Lat: %f; Lon: %f; Airspeed: %f; Altitude AGL: %f; Accel-z: %f; Vel-z_ef: %f\n", deltat, pkt.timestamp, pkt.position_latitude, pkt.position_longitude, airspeed, pkt.altitude_ground, pkt.az, velocity_ef[2]);
    
    if (0) {
        printf("Delta: %f Time: %" PRIu64 "\n", deltat, time_now_us);
        printf("Accel.x %f\n", accel_body.x);
        printf("Accel.y %f\n", accel_body.y);
        printf("Accel.z %f\n", accel_body.z);
        printf("Gyro.x  %f\n", gyro.x);
        printf("Gyro.y  %f\n", gyro.y);
        printf("Gyro.z  %f\n", gyro.z);
        printf("Pos.x %f\n",   position.x);
        printf("Pos.y %f\n",   position.y);
        printf("Pos.z %f\n",   position.z);
        printf("Roll %f\n",    pkt.roll);
        printf("Pitch %f\n",   pkt.pitch);
        printf("Yaw %f\n",     pkt.yaw);
    }

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(1.0/deltat);
    }
    
    last_timestamp = pkt.timestamp;
}

/*
  update the SilentWings simulation by one time step
 */
void SilentWings::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
