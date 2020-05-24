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
 * AP_Mission_Relative.cpp
 *
 *  Created on: 08.05.2020
 *      Author: WillyZehnder
 */
/// @file    AP_Mission_Relative.cpp
/// @brief   translates and rotates Missions according to the location where Mode-AUTO has been switched on

/*
 *   The AP_Mission_Relative library:
 *   - memorizes the Location where Mode-AUTO has been switched on (Basepoint)
 *   - moves the individual Waypoints according to Homepoint, Base-Point and Parameter-Settings
 */

#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Mission_Relative.h"

AP_Mission_Relative *AP_Mission_Relative::_singleton;

const AP_Param::GroupInfo AP_Mission_Relative::var_info[] = {

    // @Param: MOVE
    // @DisplayName: kind of moving the mission
    // @Description: Defines how the mission will be translated/rotated
    // @Range: 0 5
    // @Values: 0:No move, 1:PARALLEL translation, 2:+ROTATION(1stWP-related) 3:+ROTATION(North-related) 4:+ROTATION(by Heading) 5:+ROTATION(by RC-channel)
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_MOVE",  1, AP_Mission_Relative, _kind_of_move, AP_MISSION_RELATIVE_KIND_OF_MOVE_DEFAULT, AP_PARAM_FLAG_ENABLE),

        // @Param: RADIUS
    // @DisplayName: radius to ignore translation
    // @Description: Radius around HomeLocation therein translation of Relative Mission will be ignored
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RADIUS",  2, AP_Mission_Relative, _no_translation_radius, AP_MISSION_RELATIVE_NO_TRANSLATION_RADIUS_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: selectable options for translation
    // @Description: Bitmask of what options to use in missions.
    // @Bitmask: 0: skip altitude at first Waypoint, 1: use altitude-offset (Basepoint to first Waypoint) for all Waypoints, 2: just positive altitude-offsets allowed
// @User: Advanced
    AP_GROUPINFO("OPTIONS",  3, AP_Mission_Relative, _rel_options, AP_MISSION_RELATIVE_OPTIONS_DEFAULT),

    // @Param: CHANNEL
    // @DisplayName: rc-input-channel for rotation
    // @Description: rc-input-channel to control amount of rotation 1000mys=-180deg 1500mys=0deg 2000mys=+180deg
    // @User: Advanced
    // @Values: <1:No rotation >0:selected rc-input-channel
    AP_GROUPINFO("CHANNEL",  4, AP_Mission_Relative, _rotate_ch, 0),

    AP_GROUPEND
};






// constructor

/*AP_Mission_Relative::AP_Mission_Relative(const SoaringController &parms) :
        soarparm(parms)
{
*/
AP_Mission_Relative::AP_Mission_Relative(void) {

    AP_Param::setup_object_defaults(this, var_info); // load parameter defaults

    _singleton = this;

    restart_behaviour = Restart_Behaviour::RESTART_NOT_TRANSLATED;
};

void AP_Mission_Relative::memorize()
{
//    gcs().send_text(MAV_SEVERITY_DEBUG, "soarparm.soar_active=%d", soarparm.soar_active);

    if ((RC_Channels::get_radio_in(5-1) > 1700) && _translation.calculated) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "Mission_Relative: Soaring -> NO displacement");
        return;
    }

    switch (_kind_of_move) { // fix the behaviour at restart of the Mission
    case 0:
        restart_behaviour = Restart_Behaviour::RESTART_NOT_TRANSLATED;
        break;
    case 1:
        restart_behaviour = Restart_Behaviour::RESTART_PARALLEL_TRANSLATED;
        gcs().send_text(MAV_SEVERITY_INFO, "Restart Parallel Translated");
        break;
    case 2:
        restart_behaviour = Restart_Behaviour::RESTART_ROTATED_FIRST_WP;
        gcs().send_text(MAV_SEVERITY_INFO, "Restart Rotated related 1st WP");
        break;
    case 3:
        restart_behaviour = Restart_Behaviour::RESTART_ROTATED_NORTH;
        gcs().send_text(MAV_SEVERITY_INFO, "Restart Rotated related North");
        break;
    case 4:
        restart_behaviour = Restart_Behaviour::RESTART_ROTATED_HEADING;
        gcs().send_text(MAV_SEVERITY_INFO, "Restart Rotated by Heading");
        break;
    case 5:
        restart_behaviour = Restart_Behaviour::RESTART_ROTATED_CHANNEL;
        gcs().send_text(MAV_SEVERITY_INFO, "Restart Rotated by RC-Channel");
        break;
    default:
        gcs().send_text(MAV_SEVERITY_NOTICE, "MIS__REL_MOVE out of range: %i", (int)_kind_of_move);
        restart_behaviour = Restart_Behaviour::RESTART_NOT_TRANSLATED;
    }

    if (restart_behaviour == Restart_Behaviour::RESTART_NOT_TRANSLATED) {
        _translation.do_translation = false;
        return;
    }
    else
    {
        _rel_options_fixed = _rel_options;
        _translation.calculated = false;
        _translation.do_translation = true;

        // memorize the Basepoint (location of switching to AUTO)
        AP::ahrs().get_position(_basepoint_loc); // _basepoint_loc.alt is absolute in [cm] in every case

        AP_Mission::Mission_Command tmp;

        // check if one of the Waypoints has (terrain_alt == 1) -> no translation allowed
        uint16_t i=0;
        while (AP::mission()->get_next_nav_cmd(i, tmp) && (_translation.do_translation) && (tmp.id != MAV_CMD_DO_LAND_START)) { // advance to the next command
            // check if Waypoint with Location
            if (!(tmp.content.location.lat == 0 && tmp.content.location.lng == 0)) {
                switch (tmp.id) { // no translation for TAKEOFF or LAND commands
                    case MAV_CMD_NAV_TAKEOFF:
                    case MAV_CMD_NAV_LAND:
                    case MAV_CMD_NAV_VTOL_TAKEOFF:
                    case MAV_CMD_NAV_VTOL_LAND:
                        break;
                    default:
                        if (tmp.content.location.terrain_alt == 1) {
                            _translation.do_translation = false;
                            gcs().send_text(MAV_SEVERITY_NOTICE, "Mission_Relative: terrain_alt -> NO MOVE POSSIBLE");
                            return;
                        }
                }
            }
            i++;// move on to next command
        }


        // no translation if the distance to Homepoint is too small
        tmp.content.location = AP::ahrs().get_home();
        float tmp_distance = tmp.content.location.get_distance(_basepoint_loc); // in [m]
        if (tmp_distance < _no_translation_radius) {
             gcs().send_text(MAV_SEVERITY_NOTICE, "distance to Home: %fm -> NO translation", tmp_distance);
            _translation.do_translation = false;
            return;
        }

        // calculate amount of rotation
        switch (restart_behaviour) {
            case Restart_Behaviour::RESTART_ROTATED_NORTH:
            case Restart_Behaviour::RESTART_ROTATED_FIRST_WP:
                tmp.content.location = AP::ahrs().get_home();
                // get direction from Homepoint to Basepoint (North-related)
                _translation.direction = tmp.content.location.get_bearing_to(_basepoint_loc);   // in centidegrees from 0 36000
                break;
            case Restart_Behaviour::RESTART_ROTATED_HEADING:
                // rotation via heading at Basepoint
                _translation.direction = AP::ahrs().yaw_sensor;
                break;
            case Restart_Behaviour::RESTART_ROTATED_CHANNEL:
                // rotation via RC-channel: 1000mys<->-180deg 1500mys<->0deg 2000mys<->+180deg
                uint16_t radio_in;
                if (_rotate_ch > 0) {
                    radio_in = RC_Channels::get_radio_in(_rotate_ch-1);
                    radio_in = (radio_in > 2000 ? 2000 : radio_in);
                    radio_in = (radio_in < 1000 ? 1000 : radio_in);
                    _translation.direction = 36 * (radio_in-1000) - 18000;
                }
                else {
                    _translation.direction = 0;
                }
                break;
            case Restart_Behaviour::RESTART_NOT_TRANSLATED:
            case Restart_Behaviour::RESTART_PARALLEL_TRANSLATED:
            default:
                _translation.direction = 0;
        }

        // altitude of Homepoint, necessary for calculation of altitude-adaption
        _translation.alt = tmp.content.location.alt;
    }
}

void AP_Mission_Relative::set_no_translation()
{
    _translation.do_translation = false;
}

void AP_Mission_Relative::moveloc(Location& loc, uint16_t id)
{

    //  calculate and do translation/rotation
    if (_translation.do_translation) { // no translation if generally off by parameter or if we are behind DO_LAND_START
        switch (id) { // no translation for TAKEOFF or LAND commands
            case MAV_CMD_NAV_TAKEOFF:
            case MAV_CMD_NAV_LAND:
            case MAV_CMD_NAV_VTOL_TAKEOFF:
            case MAV_CMD_NAV_VTOL_LAND:
                break;
            default:

                // check, if WP with terrain_alt == 1 has been loaded on the fly
                if (loc.terrain_alt == 1) {
                    AP::ahrs().get_position(loc); // skip Waypoint
                    gcs().send_text(MAV_SEVERITY_NOTICE, "Mission_Relative: terrain_alt -> WP skipped");
                    return;
                }
                // calculate parallel translation and corresponding values just once at first Waypoint
                if ((!_translation.calculated)&&(restart_behaviour >= Restart_Behaviour::RESTART_PARALLEL_TRANSLATED)) {

                    _translation.calculated = true;

                    _first_wp_loc = loc; // memorize not translated position of very first WayPoint

                    if (restart_behaviour == Restart_Behaviour::RESTART_ROTATED_FIRST_WP) {
                        // direction from Homepoint to untranslated 1st-Waypoint (1st-WP-related)
                        _translation.direction -= AP::ahrs().get_home().get_bearing_to(_first_wp_loc);
                    }

                    // calculate altitude-translation
                    if (loc.relative_alt == 1) {
                        _translation.alt = _basepoint_loc.alt - loc.alt - _translation.alt; // subtract Home-Altitude if WP-altitude is relative to
                    } else {
                        _translation.alt = _basepoint_loc.alt - loc.alt;
                    }

                    // allow just positive offsets for altitude
                    if ((AP_MISSION_RELATIVE_MASK_POSITIVE_ALT_OFFSET & _rel_options_fixed) && (_translation.alt < 0)) {
                        _translation.alt = 0;
                    }

                    loc.lat = _basepoint_loc.lat; // put 1st-Waypoint to Basepoint as basically defined
                    loc.lng = _basepoint_loc.lng;

                    // skip first Waypoint by setting nominal location to actual (esp. altitude)
                    if ((AP_MISSION_RELATIVE_MASK_SKIP_FIRST_WP & _rel_options_fixed) || (AP_MISSION_RELATIVE_MASK_USE_ALT_OFFSET & _rel_options_fixed)){
                        AP::ahrs().get_position(loc); // skip Waypoint
                    }
                } else {
                    if (restart_behaviour >= Restart_Behaviour::RESTART_PARALLEL_TRANSLATED){ // do at least parallel translation
                        AP_Mission_Relative::translate(loc);
                    }
                    if (restart_behaviour > Restart_Behaviour::RESTART_PARALLEL_TRANSLATED){ // do additional rotation
                        AP_Mission_Relative::rotate(loc);
                    }
                }
        }
    }
}

void AP_Mission_Relative::translate(Location& loc)
{
    Location tmp;
    tmp = loc; // before translation
    loc.lat = _basepoint_loc.lat + (loc.lat - _first_wp_loc.lat);
    // correction of lng, based on lat-translation
    loc.lng = _basepoint_loc.lng + (loc.lng - _first_wp_loc.lng) / tmp.longitude_scale() * loc.longitude_scale();

    if (AP_MISSION_RELATIVE_MASK_USE_ALT_OFFSET & _rel_options_fixed) { // do altitude translation
        loc.alt += _translation.alt;
    }
}

void AP_Mission_Relative::rotate(Location& loc)
{
    float rel_lat, rel_lng; // position of currently parallel translated WayPoint relative to Basepoint
    float rel_distance;     // imaginary distance lat/lng from Basepoint to current WP in [10^7 Degrees]
    rel_lat =  loc.lat - _basepoint_loc.lat;
    rel_lng = (loc.lng - _basepoint_loc.lng)*loc.longitude_scale();

    Rotation tmprot;
    tmprot.direction = _basepoint_loc.get_bearing_to(loc);   // direction from Basepoint to current WP in centidegrees (not rotated)
    tmprot.direction = _translation.direction + tmprot.direction; // total rotation direction in centidegrees
    if (tmprot.direction < 0) {
        tmprot.direction += 36000;
    }
    rel_distance = sqrtf((rel_lat*rel_lat)+(rel_lng*rel_lng));
    tmprot.lat = (int32_t)(cosf((float)tmprot.direction/100.0*DEG_TO_RAD)*rel_distance);
    tmprot.lng = (int32_t)(sinf((float)tmprot.direction/100.0*DEG_TO_RAD)*rel_distance)/loc.longitude_scale();
    loc.lat = _basepoint_loc.lat + tmprot.lat;
    loc.lng = _basepoint_loc.lng + tmprot.lng;

}

bool AP_Mission_Relative::check_reason_valid(ModeReason reason)
{
    return true;
}

namespace AP {
    AP_Mission_Relative &mission_relative() {
        return *AP_Mission_Relative::get_singleton();
    }
}
