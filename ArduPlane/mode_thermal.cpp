#include "mode.h"
#include "Plane.h"

bool ModeThermal::_enter()
{
#if SOARING_ENABLED != ENABLED
    return false;
#else
    if (!plane.g2.soaring_controller.is_active()) {
        return false;
    }    
    
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.loiter_angle_reset();

    plane.g2.soaring_controller.init_thermalling();
    plane.g2.soaring_controller.get_target(plane.next_WP_loc); // ahead on flight path

    return true;
#endif
}

void ModeThermal::update()
{
#if SOARING_ENABLED == ENABLED

    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();

    // Update thermal estimate and check for switch back to AUTO
    plane.g2.soaring_controller.update_thermalling();  // Update estimate

    // Thermalling is done in a home-relative coordinate system, so we need home to be set.
    Vector3f position;
    if (!AP::ahrs().get_relative_position_NED_home(position)) {
        return;
    }

    // Check distance to home against MAX_RADIUS.
    if (plane.g2.soaring_controller.max_radius >= 0 &&
        sq(position.x)+sq(position.y) > sq(plane.g2.soaring_controller.max_radius) &&
        plane.previous_mode->mode_number()!=Mode::Number::AUTO) {
        // Some other loiter status, and outside of maximum soaring radius, and previous mode wasn't AUTO
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Outside SOAR_MAX_RADIUS, RTL");
        plane.set_mode(plane.mode_rtl, ModeReason::SOARING_DRIFT_EXCEEDED);
        return;
    }

    // If previous mode was AUTO and there was a previous NAV command&, we can use previous and next wps for drift calculation
    // with respect to the desired direction of travel. If these vectors are zero, drift will be calculated from thermal start
    // position only, without taking account of the desired direction of travel.
    Vector2f prev_wp, next_wp;

    if (plane.previous_mode == &plane.mode_auto) {
        AP_Mission::Mission_Command current_nav_cmd = plane.mission.get_current_nav_cmd();
        AP_Mission::Mission_Command prev_nav_cmd;

        if (!(plane.mission.get_next_nav_cmd(plane.mission.get_prev_nav_cmd_with_wp_index(), prev_nav_cmd) &&
            prev_nav_cmd.content.location.get_vector_xy_from_origin_NE(prev_wp) &&
            current_nav_cmd.content.location.get_vector_xy_from_origin_NE(next_wp))) {
            prev_wp.zero();
            next_wp.zero();
        }
    }

    // Get the status of the soaring controller cruise checks.
    const SoaringController::LoiterStatus loiterStatus = plane.g2.soaring_controller.check_cruise_criteria(prev_wp/100, next_wp/100);

    if (loiterStatus == SoaringController::LoiterStatus::GOOD_TO_KEEP_LOITERING) {
        // Reset loiter angle, so that the loiter exit heading criteria
        // only starts expanding when we're ready to exit.
        plane.loiter.sum_cd = 0;
        plane.soaring_mode_timer_ms = AP_HAL::millis();

        //update the wp location
        plane.g2.soaring_controller.get_target(plane.next_WP_loc);

        return;
    }

    // Some other loiter status, we need to think about exiting loiter.
    const uint32_t time_in_loiter_ms = AP_HAL::millis() - plane.soaring_mode_timer_ms;

    if (!exit_heading_aligned() && loiterStatus!=SoaringController::LoiterStatus::ALT_TOO_LOW && time_in_loiter_ms<20000) {
        // Heading not lined up, and not timed out or in a condition requiring immediate exit.
        return;
    }

    // Heading lined up and loiter status not good to continue. Need to switch mode.

    // Determine appropriate mode.
    Mode* exit_mode = plane.previous_mode;

    if (loiterStatus == SoaringController::LoiterStatus::ALT_TOO_LOW && 
       ((plane.previous_mode == &plane.mode_cruise) || (plane.previous_mode == &plane.mode_fbwb))) {
        exit_mode = &plane.mode_rtl;
    }

    // Print message and set mode.
    switch (loiterStatus) {
    case SoaringController::LoiterStatus::ALT_TOO_HIGH:
        restore_mode("Too high", ModeReason::SOARING_ALT_TOO_HIGH, *exit_mode);
        break;
    case SoaringController::LoiterStatus::ALT_TOO_LOW:
        restore_mode("Too low", ModeReason::SOARING_ALT_TOO_LOW, *exit_mode);
        break;
    default:
    case SoaringController::LoiterStatus::THERMAL_WEAK:
        restore_mode("Thermal ended", ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED, *exit_mode);
        break;
    case SoaringController::LoiterStatus::DRIFT_EXCEEDED:
        restore_mode("Drifted too far", ModeReason::SOARING_DRIFT_EXCEEDED, *exit_mode);
        break;
    } // switch loiterStatus
#endif // SOARING_ENABLED
}

bool ModeThermal::exit_heading_aligned() const
{
    // Return true if the current heading is aligned with the next objective.
    // If home is not set, or heading not locked, return true to avoid delaying mode change.
    switch (plane.previous_mode->mode_number()) {
    case Mode::Number::AUTO: {
        //Get the lat/lon of next Nav waypoint after this one:
        AP_Mission::Mission_Command current_nav_cmd = plane.mission.get_current_nav_cmd();;
        return plane.mode_loiter.isHeadingLinedUp(plane.next_WP_loc, current_nav_cmd.content.location);
    }
    case Mode::Number::FLY_BY_WIRE_B:
        return (!AP::ahrs().home_is_set() || plane.mode_loiter.isHeadingLinedUp(plane.next_WP_loc, AP::ahrs().get_home()));
    case Mode::Number::CRUISE:
        return (!plane.cruise_state.locked_heading || plane.mode_loiter.isHeadingLinedUp_cd(plane.cruise_state.locked_heading_cd));
    default:
        break;
    }
    return true;
}

void ModeThermal::restore_mode(const char *reason, ModeReason modereason, Mode &exit_mode)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Soaring: %s, restoring %s", reason, exit_mode.name());
    plane.set_mode(exit_mode, modereason);
}

void ModeThermal::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}
