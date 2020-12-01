#include "AP_AltitudePlanner.h"

void AP_AltitudePlanner::setup_glide_slope(const Location &start_loc, const Location &target_loc, bool apply_gradual_climb, int32_t start_adj_rel_alt, bool force_glideslope)
{
    if (apply_gradual_climb) {
        // RTL, AVOID_ADSB or GUIDED modes
        if (above_location(start_loc, target_loc)) {
            set_offset_altitude_location(target_loc, force_glideslope);
        } else {
            reset_offset_altitude();
        }
    } else {
        // AUTO mode
        if (start_adj_rel_alt > 2000 || above_location(start_loc, target_loc)) {
            set_offset_altitude_location(target_loc, force_glideslope);
        } else {
            reset_offset_altitude();
        }
    }
}

void AP_AltitudePlanner::set_target_altitude_current(const Location &loc)
{
    // record altitude above sea level at the current time as our
    // target altitude
    _target_amsl_cm = loc.alt;

    // reset any glide slope offset
    reset_offset_altitude();

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (_aparm.terrain_follow && _terrain->height_above_terrain(terrain_altitude, true)) {
        _target_terrain_following = true;
        _target_terrain_alt_cm = terrain_altitude*100;
    } else {
        // if terrain following is disabled, or we don't know our
        // terrain altitude when we set the altitude then don't
        // terrain follow
        _target_terrain_following = false;        
    }
#endif
}

void AP_AltitudePlanner::set_target_altitude_current_adjusted(const Location &loc)
{
    set_target_altitude_current(loc);

    // use adjusted_altitude_cm() to take account of ALTITUDE_OFFSET
    _target_amsl_cm = loc.alt - (mission_alt_offset()*100);
}


void AP_AltitudePlanner::set_target_altitude_location(const Location &loc)
{
    _target_amsl_cm = loc.alt;

    if (loc.relative_alt) {
        _target_amsl_cm += AP::ahrs().get_home().alt;
    }
#if AP_TERRAIN_AVAILABLE
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (loc.terrain_alt && _terrain->height_above_terrain(height, true)) {
        _target_terrain_following = true;
        _target_terrain_alt_cm = loc.alt;
        if (!loc.relative_alt) {
            // it has home added, remove it
            _target_terrain_alt_cm -= AP::ahrs().get_home().alt;
        }
    } else {
        _target_terrain_following = false;
    }
#endif
}

int32_t AP_AltitudePlanner::relative_target_altitude_cm(float lookahead_adjustment, float rangefinder_correction)
{
#if AP_TERRAIN_AVAILABLE
    float relative_home_height;
    if (_target_terrain_following && 
        _terrain->height_relative_home_equivalent(_target_terrain_alt_cm*0.01f,
                                                relative_home_height, true)) {
        // add lookahead adjustment the target altitude
        _target_lookahead = lookahead_adjustment;
        relative_home_height += _target_lookahead;

        // correct for rangefinder data
        relative_home_height += rangefinder_correction;

        // we are following terrain, and have terrain data for the
        // current location. Use it.
        return relative_home_height*100;
    }
#endif
    int32_t relative_alt = _target_amsl_cm - AP::ahrs().get_home().alt;
    relative_alt += mission_alt_offset()*100;
    relative_alt += rangefinder_correction * 100;
    return relative_alt;
}

void AP_AltitudePlanner::change_target_altitude(int32_t change_cm)
{
    _target_amsl_cm += change_cm;
#if AP_TERRAIN_AVAILABLE
    if (_target_terrain_following) {
        _target_terrain_alt_cm += change_cm;
    }
#endif
}

void AP_AltitudePlanner::set_target_altitude_proportion(const Location &loc, float proportion, int32_t altitude_error_cm, bool suppress_glideslope)
{
    set_target_altitude_location(loc);
    proportion = constrain_float(proportion, 0.0f, 1.0f);
    change_target_altitude(-_target_offset_cm*proportion);
    //rebuild the glide slope if we are above it and supposed to be climbing
    if(_aparm.glide_slope_threshold > 0) {
        if(_target_offset_cm > 0 && altitude_error_cm < -100 * _aparm.glide_slope_threshold) {
            set_target_altitude_location(loc);
            set_offset_altitude_location(loc, suppress_glideslope);
            change_target_altitude(-_target_offset_cm*proportion);
            //adjust the new target offset altitude to reflect that we are partially already done
            if(proportion > 0.0f)
                _target_offset_cm = ((float)_target_offset_cm)/proportion;
        }
    }
}


void AP_AltitudePlanner::constrain_target_altitude_location(const Location &loc1, const Location &loc2)
{
    if (loc1.alt > loc2.alt) {
        _target_amsl_cm = constrain_int32(_target_amsl_cm, loc2.alt, loc1.alt);
    } else {
        _target_amsl_cm = constrain_int32(_target_amsl_cm, loc1.alt, loc2.alt);
    }
}

int32_t AP_AltitudePlanner::calc_altitude_error_cm(Location loc)
{
#if AP_TERRAIN_AVAILABLE
    float terrain_height;
    if (is_terrain_following() && 
        _terrain->height_above_terrain(terrain_height, true)) {
        return _target_lookahead*100 + _target_terrain_alt_cm - (terrain_height*100);
    }
#endif
    return _target_amsl_cm - (loc.alt - mission_alt_offset()*100);
}


void AP_AltitudePlanner::check_fbwb_minimum_altitude(int32_t min_alt_cm)
{
    if (min_alt_cm == 0) {
        return;
    }

#if AP_TERRAIN_AVAILABLE
    if (is_terrain_following()) {
            // set our target terrain height to be at least the min set
            if (_target_terrain_alt_cm < min_alt_cm) {
                _target_terrain_alt_cm = min_alt_cm;
            }
            return;
    }
#endif

    if (_target_amsl_cm < AP::ahrs().get_home().alt + min_alt_cm) {
        _target_amsl_cm = AP::ahrs().get_home().alt + min_alt_cm;
    }
}


void AP_AltitudePlanner::reset_offset_altitude(void)
{
    _target_offset_cm = 0;
}

void AP_AltitudePlanner::set_offset_altitude_location(const Location &loc, bool force_glideslope)
{
    Location current_loc;
    AP::ahrs().get_position(current_loc);
    _target_offset_cm = loc.alt - current_loc.alt;

#if AP_TERRAIN_AVAILABLE
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (loc.terrain_alt && 
        is_terrain_following() &&
        _terrain->height_above_terrain(height, true)) {
        _target_offset_cm = _target_terrain_alt_cm - (height * 100);
    }
#endif

    if (!force_glideslope) {
        // if we are within GLIDE_SLOPE_MIN meters of the target altitude
        // then reset the offset to not use a glide slope. This allows for
        // more accurate flight of missions where the aircraft may lose or
        // gain a bit of altitude near waypoint turn points due to local
        // terrain changes
        if (_aparm.glide_slope_min <= 0 ||
            labs(_target_offset_cm)*0.01f < _aparm.glide_slope_min) {
            _target_offset_cm = 0;
        }
    }
}

bool AP_AltitudePlanner::above_location(const Location &loc1, const Location &loc2)
{
#if AP_TERRAIN_AVAILABLE
    float terrain_alt;
    if (loc2.terrain_alt && 
        _terrain->height_above_terrain(terrain_alt, true)) {
        float loc2_alt = loc2.alt*0.01f;
        if (!loc2.relative_alt) {
            loc2_alt -= AP::ahrs().get_home().alt*0.01f;
        }
        return terrain_alt > loc2_alt;
    }
#endif

    float loc2_alt_cm = loc2.alt;
    if (loc2.relative_alt) {
        loc2_alt_cm += AP::ahrs().get_home().alt;
    }
    return loc1.alt > loc2_alt_cm;
}


float AP_AltitudePlanner::mission_alt_offset(void) const {
    return _aparm.alt_offset + _additional_alt_offset;
}


float AP_AltitudePlanner::height_above_target(const Location& loc, const Location& target_loc)
{
    float target_alt = target_loc.alt*0.01;
    if (!target_loc.relative_alt) {
        target_alt -= AP::ahrs().get_home().alt*0.01f;
    }

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (target_loc.terrain_alt &&
        _terrain->height_above_terrain(terrain_altitude, true)) {
        return terrain_altitude - target_alt;
    }
#endif

    return (loc.alt*0.01f - mission_alt_offset() - AP::ahrs().get_home().alt*0.01f) - target_alt;
}
