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

#include "Plane.h"
#include <AP_AltitudePlanner/AP_AltitudePlanner.h>

/*
  altitude handling routines. These cope with both barometric control
  and terrain following control
 */

/*
  adjust altitude target depending on mode
 */
void Plane::adjust_altitude_target()
{

    // Let the altitutde planner know if terrain is disabled.
    altitudePlanner.set_terrain_disabled(terrain_disabled());
    
    Location target_location;

    if (control_mode == &mode_fbwb ||
        control_mode == &mode_cruise) {
        return;
    }
#if OFFBOARD_GUIDED == ENABLED
    if (control_mode == &mode_guided && ((guided_state.target_alt_time_ms != 0) || guided_state.target_alt > -0.001 )) { // target_alt now defaults to -1, and _time_ms defaults to zero.
        // offboard altitude demanded
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - guided_state.target_alt_time_ms);
        guided_state.target_alt_time_ms = now;
        // determine delta accurately as a float
        float delta_amt_f = delta * guided_state.target_alt_accel;
        // then scale x100 to match last_target_alt and convert to a signed int32_t as it may be negative
        int32_t delta_amt_i = (int32_t)(100.0 * delta_amt_f); 
        Location temp {};
        temp.alt = guided_state.last_target_alt + delta_amt_i; // ...to avoid floats here, 
        if (is_positive(guided_state.target_alt_accel)) {
            temp.alt = MIN(guided_state.target_alt, temp.alt);
        } else {
            temp.alt = MAX(guided_state.target_alt, temp.alt);
        }
        guided_state.last_target_alt = temp.alt;
        altitudePlanner.set_target_altitude_location(temp);
    } else 
#endif // OFFBOARD_GUIDED == ENABLED
      if (landing.is_flaring()) {
        // during a landing flare, use TECS_LAND_SINK as a target sink
        // rate, and ignores the target altitude
        altitudePlanner.set_target_altitude_location(next_WP_loc);
    } else if (landing.is_on_approach()) {
        landing.setup_landing_glide_slope(prev_WP_loc, next_WP_loc, current_loc);
        landing.adjust_landing_slope_for_rangefinder_bump(rangefinder_state, prev_WP_loc, next_WP_loc, current_loc, auto_state.wp_distance);
    } else if (landing.get_target_altitude_location(target_location)) {
       altitudePlanner.set_target_altitude_location(target_location);
#if SOARING_ENABLED == ENABLED
    } else if (g2.soaring_controller.is_active() && g2.soaring_controller.get_throttle_suppressed()) {
       // Reset target alt to current alt, to prevent large altitude errors when gliding.
       altitudePlanner.set_target_altitude_location(current_loc);
       altitudePlanner.reset_offset_altitude();
#endif
    } else if (reached_loiter_target()) {
        // once we reach a loiter target then lock to the final
        // altitude target
        altitudePlanner.set_target_altitude_location(next_WP_loc);
    } else if (altitudePlanner.get_target_offset_cm() != 0 &&
               !current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        // control climb/descent rate
        int32_t alt_error = 0; /// FIXME
        bool suppress_glideslope = flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND;
        altitudePlanner.set_target_altitude_proportion(next_WP_loc, 1.0f-auto_state.wp_proportion, alt_error, suppress_glideslope);

        // stay within the range of the start and end locations in altitude
        altitudePlanner.constrain_target_altitude_location(next_WP_loc, prev_WP_loc);
    } else {
        altitudePlanner.set_target_altitude_location(next_WP_loc);
    }

    altitude_error_cm = altitudePlanner.calc_altitude_error_cm(current_loc);

 
    float lookahead = lookahead_adjustment();
    float range_correction = rangefinder_correction();
    float target_alt = altitudePlanner.relative_target_altitude_cm(lookahead, range_correction);
}

/*
  setup for a gradual glide slope to the next waypoint, if appropriate
 */
void Plane::setup_glide_slope(void)
{
    // establish the distance we are travelling to the next waypoint,
    // for calculating out rate of change of altitude
    auto_state.wp_distance = current_loc.get_distance(next_WP_loc);
    auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    SpdHgt_Controller->set_path_proportion(auto_state.wp_proportion);
    update_flight_stage();

    /*
      work out if we will gradually change altitude, or try to get to
      the new altitude as quickly as possible.
     */
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
        /* glide down slowly if above target altitude, but ascend more
           rapidly if below it. See
           https://github.com/ArduPilot/ardupilot/issues/39
        */
        altitudePlanner.setup_glide_slope(current_loc, next_WP_loc, true, adjusted_relative_altitude_cm(), flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND);
        break;

    case Mode::Number::AUTO:
        // we only do glide slide handling in AUTO when above 20m or
        // when descending. The 20 meter threshold is arbitrary, and
        // is basically to prevent situations where we try to slowly
        // gain height at low altitudes, potentially hitting
        // obstacles.
        altitudePlanner.setup_glide_slope(current_loc, next_WP_loc, false, adjusted_relative_altitude_cm(), flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND);
        break;
    default:
        altitudePlanner.reset_offset_altitude();
        break;
    }
}

/*
  return RTL altitude as AMSL altitude
 */
int32_t Plane::get_RTL_altitude()
{
    if (g.RTL_altitude_cm < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude_cm + home.alt;
}

/*
  return relative altitude in meters (relative to terrain, if available,
  or home otherwise)
 */
float Plane::relative_ground_altitude(bool use_rangefinder_if_available)
{
   if (use_rangefinder_if_available && rangefinder_state.in_range) {
        return rangefinder_state.height_estimate;
   }

   if (use_rangefinder_if_available && quadplane.in_vtol_land_final() &&
       rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::OutOfRangeLow) {
       // a special case for quadplane landing when rangefinder goes
       // below minimum. Consider our height above ground to be zero
       return 0;
   }

#if AP_TERRAIN_AVAILABLE
    float altitude;
    if (altitudePlanner.is_terrain_following() &&
        terrain.status() == AP_Terrain::TerrainStatusOK &&
        terrain.height_above_terrain(altitude, true)) {
        return altitude;
    }
#endif

    if (quadplane.in_vtol_land_descent() &&
        !(quadplane.options & QuadPlane::OPTION_MISSION_LAND_FW_APPROACH)) {
        // when doing a VTOL landing we can use the waypoint height as
        // ground height. We can't do this if using the
        // LAND_FW_APPROACH as that uses the wp height as the approach
        // height
        return altitudePlanner.height_above_target(current_loc, next_WP_loc);
    }

    return relative_altitude;
}

/*
  reset the altitude offset used for glide slopes
 */
void Plane::reset_offset_altitude(void)
{
    altitudePlanner.reset_offset_altitude();
}


/*
  return true if current_loc is above loc. Used for glide slope
  calculations.

  "above" is simple if we are not terrain following, as it just means
  the pressure altitude of one is above the other.

  When in terrain following mode "above" means the over-the-terrain
  current altitude is above the over-the-terrain alt of loc. It is
  quite possible for current_loc to be "above" loc when it is at a
  lower pressure altitude, if current_loc is in a low part of the
  terrain
 */
bool Plane::above_location_current(const Location &loc)
{
  return altitudePlanner.above_location(current_loc, loc);
}

/*
  modify a destination to be setup for terrain following if
  TERRAIN_FOLLOW is enabled
 */
void Plane::setup_terrain_target_alt(Location &loc)
{
#if AP_TERRAIN_AVAILABLE
    if (aparm.terrain_follow) {
        loc.terrain_alt = true;
    }
#endif
}

/*
  return current_loc.alt adjusted for ALT_OFFSET
  This is useful during long flights to account for barometer changes
  from the GCS, or to adjust the flying height of a long mission
 */
int32_t Plane::adjusted_altitude_cm(void)
{
    return current_loc.alt - (altitudePlanner.mission_alt_offset()*100);
}

/*
  return home-relative altitude adjusted for ALT_OFFSET This is useful
  during long flights to account for barometer changes from the GCS,
  or to adjust the flying height of a long mission
 */
int32_t Plane::adjusted_relative_altitude_cm(void)
{
    return (relative_altitude - altitudePlanner.mission_alt_offset())*100;
}




/*
  work out target altitude adjustment from terrain lookahead
 */
float Plane::lookahead_adjustment(void)
{
#if AP_TERRAIN_AVAILABLE
    int32_t bearing_cd;
    int16_t distance;
    // work out distance and bearing to target
    if (control_mode == &mode_fbwb) {
        // there is no target waypoint in FBWB, so use yaw as an approximation
        bearing_cd = ahrs.yaw_sensor;
        distance = g.terrain_lookahead;
    } else if (!reached_loiter_target()) {
        bearing_cd = nav_controller->target_bearing_cd();
        distance = constrain_float(auto_state.wp_distance, 0, g.terrain_lookahead);
    } else {
        // no lookahead when loitering
        bearing_cd = 0;
        distance = 0;
    }
    if (distance <= 0) {
        // no lookahead
        return 0;
    }

    
    float groundspeed = ahrs.groundspeed();
    if (groundspeed < 1) {
        // we're not moving
        return 0;
    }
    // we need to know the climb ratio. We use 50% of the maximum
    // climb rate so we are not constantly at 100% throttle and to
    // give a bit more margin on terrain
    float climb_ratio = 0.5f * SpdHgt_Controller->get_max_climbrate() / groundspeed;

    if (climb_ratio <= 0) {
        // lookahead makes no sense for negative climb rates
        return 0;
    }
    
    // ask the terrain code for the lookahead altitude change
    float lookahead = terrain.lookahead(bearing_cd*0.01f, distance, climb_ratio);
    
    int32_t offset_cm = altitudePlanner.get_target_altitude_offset_cm(); 
    if (offset_cm < 0) {
        // we are heading down to the waypoint, so we don't need to
        // climb as much
        lookahead += offset_cm*0.01f;
    }

    // constrain lookahead to a reasonable limit
    return constrain_float(lookahead, 0, 1000.0f);
#else
    return 0;
#endif
}


/*
  correct target altitude using rangefinder data. Returns offset in
  meters to correct target altitude. A positive number means we need
  to ask the speed/height controller to fly higher
 */
float Plane::rangefinder_correction(void)
{
    if (millis() - rangefinder_state.last_correction_time_ms > 5000) {
        // we haven't had any rangefinder data for 5s - don't use it
        return 0;
    }

    // for now we only support the rangefinder for landing 
    bool using_rangefinder = (g.rangefinder_landing && flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND);
    if (!using_rangefinder) {
        return 0;
    }

    return rangefinder_state.correction;
}

/*
  correct rangefinder data for terrain height difference between
  NAV_LAND point and current location
 */
void Plane::rangefinder_terrain_correction(float &height)
{
#if AP_TERRAIN_AVAILABLE
    if (!g.rangefinder_landing ||
        flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND ||
        aparm.terrain_follow == 0) {
        return;
    }
    float terrain_amsl1, terrain_amsl2;
    if (!terrain.height_amsl(current_loc, terrain_amsl1, false) ||
        !terrain.height_amsl(next_WP_loc, terrain_amsl2, false)) {
        return;
    }
    float correction = (terrain_amsl1 - terrain_amsl2);
    height += correction;
    auto_state.terrain_correction = correction;
#endif
}


void Plane::set_target_altitude_current(void)
{
  altitudePlanner.set_target_altitude_current(current_loc);
}

/*
  update the offset between rangefinder height and terrain height
 */
void Plane::rangefinder_height_update(void)
{
    float distance = rangefinder.distance_cm_orient(ROTATION_PITCH_270)*0.01f;
    
    if ((rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) && ahrs.home_is_set()) {
        if (!rangefinder_state.have_initial_reading) {
            rangefinder_state.have_initial_reading = true;
            rangefinder_state.initial_range = distance;
        }
        // correct the range for attitude (multiply by DCM.c.z, which
        // is cos(roll)*cos(pitch))
        rangefinder_state.height_estimate = distance * ahrs.get_rotation_body_to_ned().c.z;

        rangefinder_terrain_correction(rangefinder_state.height_estimate);

        // we consider ourselves to be fully in range when we have 10
        // good samples (0.2s) that are different by 5% of the maximum
        // range from the initial range we see. The 5% change is to
        // catch Lidars that are giving a constant range, either due
        // to misconfiguration or a faulty sensor
        if (rangefinder_state.in_range_count < 10) {
            if (!is_equal(distance, rangefinder_state.last_distance) &&
                fabsf(rangefinder_state.initial_range - distance) > 0.05f * rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)*0.01f) {
                rangefinder_state.in_range_count++;
            }
            if (fabsf(rangefinder_state.last_distance - distance) > rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)*0.01*0.2) {
                // changes by more than 20% of full range will reset counter
                rangefinder_state.in_range_count = 0;
            }
        } else {
            rangefinder_state.in_range = true;
            if (!rangefinder_state.in_use &&
                (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND ||
                 control_mode == &mode_qland ||
                 control_mode == &mode_qrtl ||
                 (control_mode == &mode_auto && quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id))) &&
                g.rangefinder_landing) {
                rangefinder_state.in_use = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder engaged at %.2fm", (double)rangefinder_state.height_estimate);
            }
        }
        rangefinder_state.last_distance = distance;
    } else {
        rangefinder_state.in_range_count = 0;
        rangefinder_state.in_range = false;
    }

    if (rangefinder_state.in_range) {
        // base correction is the difference between baro altitude and
        // rangefinder estimate
        float correction = relative_altitude - rangefinder_state.height_estimate;

#if AP_TERRAIN_AVAILABLE
        // if we are terrain following then correction is based on terrain data
        float terrain_altitude;
        if ((altitudePlanner.is_terrain_following() || aparm.terrain_follow) &&
            terrain.height_above_terrain(terrain_altitude, true)) {
            correction = terrain_altitude - rangefinder_state.height_estimate;
        }
#endif    

        // remember the last correction. Use a low pass filter unless
        // the old data is more than 5 seconds old
        uint32_t now = millis();
        if (now - rangefinder_state.last_correction_time_ms > 5000) {
            rangefinder_state.correction = correction;
            rangefinder_state.initial_correction = correction;
            landing.set_initial_slope();
            rangefinder_state.last_correction_time_ms = now;
        } else {
            rangefinder_state.correction = 0.8f*rangefinder_state.correction + 0.2f*correction;
            rangefinder_state.last_correction_time_ms = now;
            if (fabsf(rangefinder_state.correction - rangefinder_state.initial_correction) > 30) {
                // the correction has changed by more than 30m, reset use of Lidar. We may have a bad lidar
                if (rangefinder_state.in_use) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder disengaged at %.2fm", (double)rangefinder_state.height_estimate);
                }
                memset(&rangefinder_state, 0, sizeof(rangefinder_state));
            }
        }
        
    }
}

/*
  determine if Non Auto Terrain Disable is active and allowed in present control mode
 */
bool Plane::terrain_disabled()
{
    return control_mode->allows_terrain_disable() && non_auto_terrain_disable;
}


