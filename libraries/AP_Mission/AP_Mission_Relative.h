/*
 * AP_Mission_Relative.h
 *
 *  Created on: 08.05.2020
 *      Author: WillyZehnder
 */
/// @file    AP_Mission_Relative.h
/// @brief   translates and rotates Missions according to the location where Mode-AUTO has been switched on

/*
 *   The AP_Mission_Relative library:
 *   - memorizes the Location where Mode-AUTO has been switched on (Base-Point)
 *   - moves the individual Waypoints according to Base-Point and Parameters
 */
#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <AP_Mission/AP_Mission.h> //################ ????????????????

// definitions
#ifndef AP_MISSION_RELATIVE_NO_TRANSLATION_RADIUS_DEFAULT
    // no translation of the Mission within a radius of XX m around Homepoint (it's a compromise: for Copter/Rover/Boat 5-10m, for Plane 50-100m should be sufficient)
    #define AP_MISSION_RELATIVE_NO_TRANSLATION_RADIUS_DEFAULT 50
#endif
#define AP_MISSION_RELATIVE_KIND_OF_MOVE_DEFAULT    0   // no translation
#define AP_MISSION_RELATIVE_OPTIONS_DEFAULT         0   // no special options
#define AP_MISSION_RELATIVE_MASK_SKIP_FIRST_WP  (1<<0)  // Skip first Waypoint altitude at Restart of Mission
#define AP_MISSION_RELATIVE_MASK_USE_ALT_OFFSET (1<<1)  // Use altitude offset at Restart of Mission for all Waypoints

    class AP_Mission;

/// @class    AP_Mission_Relative
/// @brief    Object managing movement of Waypoints
class AP_Mission_Relative
{

    friend class AP_Mission;

public:
    // constructor
    AP_Mission_Relative()
    {
        AP_Param::setup_object_defaults(this, var_info); // load parameter defaults
        restart_behaviour = Restart_Behaviour::RESTART_NOT_TRANSLATED;
    };
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Int8     _kind_of_move;          // defines the kind of move (translation/rotation) of the mission when entering Auto mode
    AP_Int8     _rel_options;           // bitmask options for special behaviour at translation
    AP_Float    _no_translation_radius; // distance in [m] from HomeLocation wherein a translation of a Relative Mission will be ignored

private:

    enum class Restart_Behaviour {
        RESTART_NOT_TRANSLATED,
        RESTART_PARALLEL_TRANSLATED,
        RESTART_ROTATED_TRANSLATED
    };

    // for lat/lng translation of a Relative Mission
    struct Translation {
        int32_t alt;            // altitude-displacement of location where AUTO has been switched on, relative to first waypoint [cm]
        int32_t direction;      // direction from HomePoint to the location where AUTO has been switched on  [10^2°] North=0 East=9000
        bool    do_translation; // for marking if location where AUTO has been switched is far enough from home-location
        bool    calculated;     // for marking if first waypoint is still proceeded and displacement is calculated
        };

    // for lat/lng rotation of a Relative Mission
    struct Rotation {
        int32_t lat;            // latitude-displacement of location where AUTO has been switched on relative to current translated waypoint [10^7°]
        int32_t lng;            // longitude-displacement of location where AUTO has been switched on relative to current translated waypoint [10^7°]
        int32_t direction;      // direction from the location where AUTO has been switched on to the current translated waypoint [10^2°] North=0 East=9000
        };

    // internal variables
    Restart_Behaviour restart_behaviour;    // behaviour at restart of a Mission

    struct Location         _basepoint_loc; // location where the MODE has been switched to AUTO
    struct Location         _first_wp_loc;  // original location of very first Waypoint
    struct Translation      _translation;   // info concerning the translation of a  Relative Mission

    /// memorizes the Location and Attitudes at Base-Point
    void memorize();

    /// set _translation.do_translation = false (necessary after DO_LAND_START has been detected)
    void set_no_translation();

    /// move - if the Command is a Waypoint the Location will be moved according to Parameters
    void moveloc(Location& loc, const uint16_t id);
//    void movecmd(AP_Mission::Mission_Command& cmd);

    /// translate - if the Command is a Waypoint the Location will be translated according to Parameters
    void translate(Location& loc);

    /// rotate - if the Command is a Waypoint the Location will be rotated according to Parameters
    void rotate(Location& loc);

    // check if command is a landing type command.  Asside the obvious, MAV_CMD_DO_PARACHUTE is considered a type of landing
//    bool is_landing_type_cmd(uint16_t id) const; ################################ kann ich das nutzen?

};

namespace AP {
    AP_Mission_Relative *mission_relative();
};
