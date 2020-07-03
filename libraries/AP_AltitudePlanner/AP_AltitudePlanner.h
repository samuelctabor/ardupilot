#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Terrain/AP_Terrain.h>

class AP_AltitudePlanner {
public:
    AP_AltitudePlanner(const AP_Vehicle::FixedWing &parms)
        : _aparm(parms)
    {
    }

    /* Do not allow copies */
    //AP_AltitudePlanner(const AP_AltitudePlanner &other) = delete;
    //AP_AltitudePlanner &operator=(const AP_AltitudePlanner&) = delete;

    /*
    set flag for force terrain disable.
    */
    void set_terrain_disabled(bool disabled) {_terrain_disabled = disabled; }

    /// provide pointer to terrain database
    void set_terrain(AP_Terrain* terrain_ptr) { _terrain = terrain_ptr; }

    /*
    setup for a gradual glide slope to the next waypoint, if appropriate
    */
    void setup_glide_slope(const Location &start_loc, const Location &target_loc, bool apply_gradual_climb, int32_t start_adj_rel_alt, bool land_flight_stage);

    /*
      set the target altitude to the current altitude. This is used when 
      setting up for altitude hold, such as when releasing elevator in
      CRUISE mode.
     */
    void set_target_altitude_current(const Location &loc);


    /*
      set the target altitude to the current altitude, with ALT_OFFSET adjustment
     */
    void set_target_altitude_current_adjusted(const Location &loc);

    /*
      set target altitude based on a location structure
     */
    void set_target_altitude_location(const Location &loc);

    /*
      return relative to home target altitude in centimeters. Used for
      altitude control libraries
     */
    int32_t relative_target_altitude_cm(float lookahead_adjustment, float rangefinder_correction);

    /*
    change the current target altitude by an amount in centimeters. Used
    to cope with changes due to elevator in CRUISE or FBWB
    */
    void change_target_altitude(int32_t change_cm);

    /*
    change target altitude by a proportion of the target altitude offset
    (difference in height to next WP from previous WP). proportion
    should be between 0 and 1. 

    When proportion is zero we have reached the destination. When
    proportion is 1 we are at the starting waypoint.

    Note that target_altitude is setup initially based on the
    destination waypoint
    */
    void set_target_altitude_proportion(const Location &loc, float proportion, int32_t altitude_error_cm, bool suppress_glideslope);

    /*
      constrain target altitude to be between two locations. Used to
      ensure we stay within two waypoints in altitude
     */
    void constrain_target_altitude_location(const Location &loc1, const Location &loc2);

    /*
      return error between target altitude and current altitude
     */
    int32_t calc_altitude_error_cm(Location loc);

    /*
      check for FBWB_min_altitude_cm violation
     */
    void check_fbwb_minimum_altitude(int32_t min_alt_cm);
    
    /*
    reset the altitude offset used for glide slopes
    */
    void reset_offset_altitude(void);

    /*
      reset the altitude offset used for glide slopes, based on difference
      between altitude at a destination and current altitude. If
      destination is above the current altitude then the result is
      positive.
     */
    void set_offset_altitude_location(const Location &loc, bool suppress_glideslope);

    /*
      return true if loc1 is above loc2. Used for glide slope
      calculations.

      "above" is simple if we are not terrain following, as it just means
      the pressure altitude of one is above the other.

      When in terrain following mode "above" means the over-the-terrain
      current altitude is above the over-the-terrain alt of loc. It is
      quite possible for loc1 to be "above" loc2 when it is at a
      lower pressure altitude, if loc1 is in a low part of the
      terrain
     */
    bool above_location(const Location &loc1, const Location &loc2);

        /*
      return the mission altitude offset. This raises or lowers all
      mission items. It is primarily set using the ALT_OFFSET parameter,
      but can also be adjusted by the rangefinder landing code for a
      NAV_LAND command if we have aborted a steep landing
     */
    float mission_alt_offset(void) const;

        // Get the height above the target location.
    float height_above_target(const Location& loc, const Location& target_loc);

    // Set an additional altitude offset to be applied to mission items/
    void set_additional_alt_offset(float offset) {_additional_alt_offset = offset;};

    // Clear any additional alt offst
    void clear_additional_alt_offset(void) {set_additional_alt_offset(0.0f);};

    int32_t get_target_offset_cm(void) {return _target_offset_cm;};

#if AP_TERRAIN_AVAILABLE
    bool is_terrain_following(void) {return _target_terrain_following;};
#endif

    /* 
      Return the vertical difference across current segment
    */
    int32_t get_target_altitude_offset_cm(void) {return _target_offset_cm;};

    int32_t get_target_amsl_cm(void) {return _target_amsl_cm;};
private:

    float _additional_alt_offset;

    AP_Terrain              *_terrain; // pointer to terrain source
    const AP_Vehicle::FixedWing &_aparm;


    /*
        Variables previously in target_altitude structure.
    */

    // target altitude above sea level in cm. Used for barometric
    // altitude navigation
    int32_t _target_amsl_cm;

    // Altitude difference between previous and current waypoint in
    // centimeters. Used for glide slope handling
    int32_t _target_offset_cm;

#if AP_TERRAIN_AVAILABLE
    // are we trying to follow terrain?
    bool _target_terrain_following;

    // target altitude above terrain in cm, valid if terrain_following
    // is set
    int32_t _target_terrain_alt_cm;

    // lookahead value for height error reporting
    float _target_lookahead;
#endif

    // last input for FBWB/CRUISE height control
    float _target_last_elevator_input;

    // last time we checked for pilot control of height
    uint32_t _target_last_elev_check_us;

    // flag for terrain force disabled.
    bool _terrain_disabled = false;
};