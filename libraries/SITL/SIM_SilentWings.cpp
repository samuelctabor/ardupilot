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
    sock(true)
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
    fdm_packet pkt, pkt_tmp;

    memset(&pkt,     0, sizeof(pkt));
    memset(&pkt_tmp, 0, sizeof(pkt));
    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    //while (sock.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
    uint8_t nread = sock.recv(&pkt_tmp, sizeof(pkt_tmp), 0);



    if (nread!=132) {
        return;
    }
    printf("Received %i\n", nread);    

    //while (nread == 0) {
    //    send_servos(input);
    //    nread = sock.recv(&pkt, sizeof(pkt), 100);
    //    printf("Received %i\n", nread);
    //}
    
    memcpy(&pkt,      &pkt_tmp,    2); // timestamp

    // The order of bytes in the doubles is incorrect.
    float* addr_tmp = (float*)&pkt_tmp + 2;
    float* addr     = (float*)&pkt     + 2;
    addr[0] = addr_tmp[1];
    addr[1] = addr_tmp[0];
    addr[2] = addr_tmp[3];
    addr[3] = addr_tmp[2];

    
    memcpy((char*)(&pkt)+24, (char*)(&pkt_tmp)+24, 112); // the rest
    

    //TEMP
    //memset(&pkt,     0, sizeof(pkt));
    // auto-adjust to crrcsim frame rate
    double deltat = (double)((pkt.timestamp - last_timestamp) % 65535)/1000.0f;

    if (deltat>100000.0) {
        return;
    }

    accel_body = Vector3f(pkt.ax, pkt.ay, -pkt.az - GRAVITY_MSS);
    gyro = Vector3f(radians(pkt.d_roll), radians(pkt.d_pitch), radians(pkt.d_yaw));
    

    loc2.lat = pkt.position_latitude * 1.0e7;
    loc2.lng = pkt.position_longitude * 1.0e7;

    if (loc1.lat==0.0f) {
        printf("Resetting home");
        loc1.lat = loc2.lat;
        loc1.lng = loc2.lng;
    }
    
    Vector2f posdelta = location_diff(loc1, loc2);
    position.x = posdelta.x;
    position.y = posdelta.y;
    position.z = -pkt.altitude_ground;

    airspeed = pkt.v_eas;
    airspeed_pitot = pkt.v_eas;

    dcm.from_euler(radians(pkt.roll), radians(pkt.pitch), radians(pkt.yaw));

    velocity_ef = dcm * Vector3f(pkt.vx, pkt.vy, pkt.vz); //

    time_now_us += deltat * 1.0e6;

    if (0) {
        printf("Delta: %f Time: %f\n", deltat, time_now_us);

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
