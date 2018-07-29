#include "AP_Soaring.h"
#include <GCS_MAVLink/GCS.h>
#include <stdint.h>
extern const AP_HAL::HAL& hal;


// ArduSoar parameters
const AP_Param::GroupInfo SoaringController::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Is the soaring mode enabled or not
    // @Description: Toggles the soaring mode on and off
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, SoaringController, soar_active, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: VSPEED
    // @DisplayName: Vertical v-speed
    // @Description: Rate of climb to trigger themalling speed
    // @Units: m/s
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("VSPEED", 2, SoaringController, thermal_vspeed, 0.7f),

    // @Param: Q1
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for strength
    // @Units:
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("Q1", 3, SoaringController, thermal_q1, 0.001f),

    // @Param: Q2
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for position and radius
    // @Units:
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("Q2", 4, SoaringController, thermal_q2, 0.03f),

    // @Param: R
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
    // @Units:
    // @Range: 0 10
    // @User: Advanced

    AP_GROUPINFO("R", 5, SoaringController, thermal_r, 0.45f),

    // @Param: DIST_AHEAD
    // @DisplayName: Distance to thermal center
    // @Description: Initial guess of the distance to the thermal center
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DIST_AHEAD", 6, SoaringController, thermal_distance_ahead, 5.0f),

    // @Param: MIN_THML_S
    // @DisplayName: Minimum thermalling time
    // @Description: Minimum number of seconds to spend thermalling
    // @Units: s
    // @Range: 0 32768
    // @User: Advanced
    AP_GROUPINFO("MIN_THML_S", 7, SoaringController, min_thermal_s, 20),

    // @Param: MIN_CRSE_S
    // @DisplayName: Minimum cruising time
    // @Description: Minimum number of seconds to spend cruising
    // @Units: s
    // @Range: 0 32768
    // @User: Advanced
    AP_GROUPINFO("MIN_CRSE_S", 8, SoaringController, min_cruise_s, 30),

    // @Param: POLAR_CD0
    // @DisplayName: Zero lift drag coef.
    // @Description: Zero lift drag coefficient
    // @Units:
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_CD0", 9, SoaringController, polar_CD0, 0.027),

    // @Param: POLAR_B
    // @DisplayName: Induced drag coeffient
    // @Description: Induced drag coeffient
    // @Units:
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_B", 10, SoaringController, polar_B, 0.031),

    // @Param: POLAR_K
    // @DisplayName: Cl factor
    // @Description: Cl factor 2*m*g/(rho*S)
    // @Units: m.m/s/s
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_K", 11, SoaringController, polar_K, 25.6),

    // @Param: ALT_MAX
    // @DisplayName: Maximum soaring altitude, relative to the home location
    // @Description: Don't thermal any higher than this.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MAX", 12, SoaringController, alt_max, 350.0),

    // @Param: ALT_MIN
    // @DisplayName: Minimum soaring altitude, relative to the home location
    // @Description: Don't get any lower than this.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MIN", 13, SoaringController, alt_min, 50.0),

    // @Param: ALT_CUTOFF
    // @DisplayName: Maximum power altitude, relative to the home location
    // @Description: Cut off throttle at this alt.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_CUTOFF", 14, SoaringController, alt_cutoff, 250.0),
    
    // @Param: ENABLE_CH
    // @DisplayName: (Optional) RC channel that toggles the soaring controller on and off
    // @Description: Toggles the soaring controller on and off. This parameter has any effect only if SOAR_ENABLE is set to 1 and this parameter is set to a valid non-zero channel number. When set, soaring will be activated when RC input to the specified channel is greater than or equal to 1700.
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("ENABLE_CH", 15, SoaringController, soar_active_ch, 0),

    // @Param: POMDP_ON
    // @DisplayName: Is the POMDSoar algorithm on?
    // @Description: If 1, the soaring controller uses the POMDSoar algorithm. If 0, the soaring controller uses the ArduSoar algorithm.
    // @Units: boolean
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("POMDP_ON", 16, SoaringController, pomdp_on, 0),

    // @Param: POMDP_N
    // @DisplayName: Number of samples per action trajectory used by POMDSoar
    // @Description: Number of samples per action trajectory used by POMDSoar.
    // @Units: samples
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POMDP_N", 17, SoaringController, _pomdsoar.pomdp_n, 10),

    // @Param: POMDP_K
    // @DisplayName: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.
    // @Description: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.
    // @Units: samples
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POMDP_K", 18, SoaringController, _pomdsoar.pomdp_k, 5),

    // @Param: POMDP_HORI
    // @DisplayName: POMDP planning horizon used by POMDSoar.
    // @Description: POMDP planning horizon used by POMDSoar.
    // @Units: seconds
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_HORI", 19, SoaringController, _pomdsoar.pomdp_hori, 4.0),

    // @Param: GPS_SYNC
    // @DisplayName: Enable synchronization between vario updates and GPS updates.
    // @Description: Enable synchronization between vario updates and GPS updates. 0 = off, 1 = sync vario update with GPS update.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("GPS_SYNC", 21, SoaringController, gps_sync, 1),

    // @Param: POMDP_STEP_T
    // @DisplayName:POMDP planning step solve time
    // @Description: The amount of computation time the POMDP solver has for computing the next action
    // @Units: seconds
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_STEP", 22, SoaringController, _pomdsoar.pomdp_step_t, 1),

    // @Param: POMDP_LOOP
    // @DisplayName: Number of POMDP solver's inner loop executions per planning step
    // @Description: Number of POMDP solver's inner loop executions per planning step (see also the POMDP_STEP_T parameter)
    // @Units:
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_LOOP", 23, SoaringController, _pomdsoar.pomdp_loop_load, 1),

    // @Param: POMDP_ROLL1
    // @DisplayName: POMDP's maximum commanded roll angle.
    // @Description: Maximum commanded roll angle in the POMDP used by POMDSoar.
    // @Units: degrees
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_ROLL1", 25, SoaringController, _pomdsoar.pomdp_roll1, 15),

    // @Param: POMDP_ROLL2
    // @DisplayName: POMDP's minimum commanded roll angle.
    // @Description: Minimum commanded roll angle in the POMDP used by POMDSoar.
    // @Units: degrees
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_ROLL2", 26, SoaringController, _pomdsoar.pomdp_roll2, 45),

    // @Param: POMDP_RRATE
    // @DisplayName: The sailplane UAV's roll rate increment used by POMDSoar
    // @Description: The sailplane UAV's roll rate increment used by POMDSoar.
    // @Units: degrees/second
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_RRATE", 27, SoaringController, _pomdsoar.pomdp_roll_rate, 75),

    // @Param: POMDP_N_ACT
    // @DisplayName: POMDP number of actions
    // @Description: Number of actions in the POMDP used by POMDSoar. The roll angle input commands corresponding to actions are endpoints of (POMDP_N_ACT-1) equal intervals between POMDP_ROLL2 and POMDP_ROLL1 (inclusive).
    // @Units: seconds
    // @Range: 1 254
    // @User: Advanced
    AP_GROUPINFO("POMDP_N_ACT", 28, SoaringController, _pomdsoar.pomdp_n_actions, 2),

    // @Param: I_MOMENT
    // @DisplayName: I-moment coefficient
    // @Description: Airframe-specific I-moment coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("I_MOMENT", 29, SoaringController, _pomdsoar.I_moment, 0.00257482),

    // @Param: K_AILERON
    // @DisplayName: Aileron K coefficient
    // @Description: Airframe-specific aileron K coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units: seconds
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("K_AILERON", 30, SoaringController, _pomdsoar.k_aileron, 1.44833047),

    // @Param: K_ROLLDAMP
    // @DisplayName: Roll dampening K coefficient
    // @Description: Airframe-specific roll-dampening K coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("K_ROLLDAMP", 31, SoaringController, _pomdsoar.k_roll_damping, 0.41073589),

    // @Param: ROLL_CLP
    // @DisplayName: CLP coefficient
    // @Description: Airframe-specific CLP coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("ROLL_CLP", 32, SoaringController, _pomdsoar.c_lp, -1.12808702679),

    // @Param: POLY_A
    // @DisplayName: Sink polynomial coefficient a
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_A", 34, SoaringController, poly_a, -0.03099261),

    // @Param: POLY_B
    // @DisplayName: Sink polynomial coefficient b
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_B", 35, SoaringController, poly_b, 0.44731854),

    // @Param: POLY_C
    // @DisplayName: Sink polynomial coefficient c
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_C", 36, SoaringController, poly_c, -2.30292972),

    // @Param: POMDP_TH
    // @DisplayName: POMDSoar's threshold on tr(P) for switching between explore and max-lift modes.
    // @Description: POMDSoar's threshold on the P matrix trace for switching between explore and max-lift modes.
    // @Units:
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("POMDP_PTH", 37, SoaringController, _pomdsoar.pomdp_pth, 50),

    // @Param: ASPD_SRC
    // @DisplayName: Airspeed source
    // @Description: 0 = airspeed sensor, 1 = wind corrected ground speed
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ASPD_SRC", 38, SoaringController, aspd_src, 1),

    // @Param: EXIT_MODE
    // @DisplayName: Thermal exit mode
    // @Description: Thermal exit mode. 0 = ArduSoar, 1 (recommended) or 2 = POMDP. It's possible to use ArduSoar's thermal exit mode with POMDSoar, but ArduSoar can only use its own thermal exit mode, 0.
    // @Units:
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("EXIT_MODE", 39, SoaringController, exit_mode, 0),

    // @Param: POMDP_NORM
    // @DisplayName: Normalize the P matrix trace when solving the POMDP
    // @Description: Normalize the trace of the P matrix used for switching between explore and max-lift modes in POMDSoar. 0 = no normalizing, 1 = normalize tr(P)
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("POMDP_NORM", 40, SoaringController, _pomdsoar.pomdp_norm_pth, 0),

    // @Param: POMDP_EXT
    // @DisplayName: Enable action duration extension in POMDSoar's max-lift mode compared to the explore mode
    // @Description: 0 = off, > 1 = multiplicative factor by which to extend action duration in max-lift compared to the explore mode.
    // @Units:
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("POMDP_EXT", 45, SoaringController, _pomdsoar.pomdp_extend, 0),

    // @Param: POMDP_PLN
    // @DisplayName: Enable deterministic trajectory planning mode for the POMDP
    // @Description: Enable deterministic trajectory planning mode for the POMDP. 0 = off, 1 on.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("POMDP_PLN", 46, SoaringController, _pomdsoar.pomdp_plan_mode, 0),

    // @Param: RUN_TEST
    // @DisplayName: Run a timing test
    // @Description: 0 = off, 1 = exp test
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RUN_TEST", 47, SoaringController, run_timing_test, 0),

    // @Param: ARSP_CMD
    // @DisplayName: Commanded airspeed
    // @Description: Commanded airspeed.
    // @Units: m/s
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("ARSP_CMD", 48, SoaringController, aspd_cmd, 9),

    // @Param: TEST_DIST
    // @DisplayName: Initial distance to the test thermal
    // @Description: Initial distance to the test thermal's center from the UAV along the UAV's (straight) path.
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("TEST_DIST", 49, SoaringController, test_dist, 20),

    // @Param: TEST_OFFSET
    // @DisplayName: Offset of the test thermal
    // @Description: Offset of the test thermal from the UAV's path.
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("TEST_OFFSET", 50, SoaringController, test_offset, 10),

    // @Param: TEST_RADIUS
    // @DisplayName: Radius of the test thermal
    // @Description: Radius of the test thermal.
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("TEST_RADIUS", 51, SoaringController, test_radius, 20),

    // @Param: TEST_STRENGTH
    // @DisplayName: Strength of the test thermal
    // @Description: Strength of the test thermal.
    // @Units: m/s
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("TEST_W", 52, SoaringController, test_strength, 2.5),

    AP_GROUPEND
};


SoaringController::SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl &spdHgt, const AP_Vehicle::FixedWing &parms, AP_RollController  &rollController, AP_Float &scaling_speed) :
    _ahrs(ahrs),
    _spdHgt(spdHgt),
    _aparm(parms),
    _vario(ahrs,spdHgt,parms),
    _loiter_rad(parms.loiter_radius),
    _throttle_suppressed(true),
    _gps(ahrs.get_gps()),
    _pomdsoar(this, rollController, scaling_speed),
    _prev_run_timing_test(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    _prev_update_time = AP_HAL::micros64();
    VectorN<float, 3> X = (const float[]) { 0, 0, 0 };
    MatrixN<float, 3> P = (const float[]) { 0.5, 0.5, 0.5 };
    MatrixN<float, 3> Q = (const float[]) { 0.0001, 0.001, 0.001 };
    float R = 0.5;
    _wind_ekf.reset(X, P, Q, R);
}

void SoaringController::get_target(Location &wp)
{
    if (_pomdsoar.are_computations_in_progress()) {
        // Do nothing -- POMDSoar doesn't use waypoints, and doesn't modify them. Therefore, don't change wp's value
    }
    else {
        wp = _prev_update_location;
        location_offset(wp, _ekf.X[2], _ekf.X[3]);
    }
}

bool SoaringController::suppress_throttle()
{
    float alt = 0;
    get_altitude_wrt_home(&alt);

    if (_throttle_suppressed && (alt < alt_min)) {
        // Time to throttle up
        _throttle_suppressed = false;
    } else if ((!_throttle_suppressed) && (alt > alt_cutoff)) {
        // Start glide
        _throttle_suppressed = true;
        // Zero the pitch integrator - the nose is currently raised to climb, we need to go back to glide.
        _spdHgt.reset_pitch_I();
        _cruise_start_time_us = AP_HAL::micros64();
        // Reset the filtered vario rate - it is currently elevated due to the climb rate and would otherwise take a while to fall again,
        // leading to false positives.
        _vario.filtered_reading = 0;
    }

    return _throttle_suppressed;
}

bool SoaringController::check_thermal_criteria()
{
    return (soar_active
            && ((AP_HAL::micros64() - _cruise_start_time_us) > ((unsigned)min_cruise_s * 1e6))
            && _vario.filtered_reading > thermal_vspeed
            && _vario.alt < alt_max
            && _vario.alt > alt_min);
}

bool SoaringController::check_cruise_criteria()
{
    float thermalability = -1e6;
    float alt = _vario.alt;

    if (pomdp_on && (exit_mode == 1 || exit_mode == 2)) {
        thermalability = _pomdsoar.assess_thermalability(uint8_t(exit_mode));
    } else {
        _loiter_rad = _aparm.loiter_radius;
        thermalability = (_ekf.X[0] * expf(-powf(_loiter_rad / _ekf.X[1], 2))) - EXPECTED_THERMALLING_SINK;
    }

    if (soar_active && (AP_HAL::micros64() - _thermal_start_time_us) > ((unsigned)min_thermal_s * 1e6) && thermalability < McCready(alt)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Thermal weak, recommend quitting: W %f R %f th %f alt %f Mc %f\n", (double)_ekf.X[0], (double)_ekf.X[1], (double)thermalability, (double)alt, (double)McCready(alt));
        return true;
    } else if (soar_active && (alt>alt_max || alt<alt_min)) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Out of allowable altitude range, beginning cruise. Alt = %f\n", (double)alt);
        return true;
    }

    return false;
}


bool SoaringController::is_in_thermal_locking_period()
{
    return ((AP_HAL::micros64() - _thermal_start_time_us) < ((unsigned)min_thermal_s * 1e6));
}


void SoaringController::init_ekf()
{
    // Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
    float r = powf(thermal_r, 2);
    float cov_q1 = powf(thermal_q1, 2); // State covariance
    float cov_q2 = powf(thermal_q2, 2); // State covariance
    const float init_q[4] = {cov_q1, cov_q2, cov_q2, cov_q2};
    const MatrixN<float,4> q{init_q};
    const float init_p[4] = {INITIAL_STRENGTH_COVARIANCE, INITIAL_RADIUS_COVARIANCE, INITIAL_POSITION_COVARIANCE, INITIAL_POSITION_COVARIANCE};
    const MatrixN<float,4> p{init_p};
    float ground_course = radians(_ahrs.get_gps().ground_course());
    float head_sin = sinf(ground_course); //sinf(_ahrs.yaw);
    float head_cos = cosf(ground_course); //cosf(_ahrs.yaw);

    // New state vector filter will be reset. Thermal location is placed in front of a/c
    const float init_xr[4] = {INITIAL_THERMAL_STRENGTH,
                              INITIAL_THERMAL_RADIUS,
                              thermal_distance_ahead * head_cos,
                              thermal_distance_ahead * head_sin };
    const VectorN<float,4> xr{init_xr};

    // Also reset covariance matrix p so filter is not affected by previous data
    _ekf.reset(xr, p, q, r);
}


void SoaringController::init_thermalling()
{
    _ahrs.get_position(_prev_update_location);
    _prev_update_time = AP_HAL::micros64();
    _thermal_start_time_us = AP_HAL::micros64();
    _pomdsoar.init_thermalling();
}

void SoaringController::init_cruising()
{
    if (is_active() && suppress_throttle()) {
        _cruise_start_time_us = AP_HAL::micros64();
        // Start glide. Will be updated on the next loop.
        _throttle_suppressed = true;
    }
}

void SoaringController::get_wind_corrected_drift(const Location *current_loc, const Vector3f *wind, float *wind_drift_x, float *wind_drift_y, float *dx, float *dy)
{
    Vector2f diff = location_diff(_prev_update_location, *current_loc); // get distances from previous update
    *dx = diff.x;
    *dy = diff.y;

    // Wind correction
    *wind_drift_x = wind->x * (AP_HAL::micros64() - _prev_update_time) * 1e-6;
    *wind_drift_y = wind->y * (AP_HAL::micros64() - _prev_update_time) * 1e-6;
    *dx -= *wind_drift_x;
    *dy -= *wind_drift_y;
}

void SoaringController::get_altitude_wrt_home(float *alt) const
{
    _ahrs.get_relative_position_D_home(*alt);
    *alt *= -1.0f;
}


bool SoaringController::is_set_to_continue_past_thermal_locking_period()
{
    return _pomdsoar.is_set_to_continue_past_thermal_locking_period();
}
void SoaringController::update_thermalling()
{
    struct Location current_loc;
    _ahrs.get_position(current_loc);

    if (soar_active
        && pomdp_on
        && _pomdsoar.are_computations_in_progress()
        && (is_in_thermal_locking_period() || _pomdsoar.is_set_to_continue_past_thermal_locking_period()))
    {
        bool is_ok_to_stop = _pomdsoar.update_thermalling(current_loc);

        if (is_ok_to_stop && check_cruise_criteria()) {
            _pomdsoar.stop_computations();
        }
    } else {
        _pomdsoar.stop_computations();
    }

    if (_vario.new_data) {
        float dx = 0;
        float dy = 0;
        float dx_w = 0;
        float dy_w = 0;
        Vector3f wind = _ahrs.wind_estimate();
        get_wind_corrected_drift(&current_loc, &wind, &dx_w, &dy_w, &dx, &dy);

        // write log - save the data.
        DataFlash_Class::instance()->Log_Write("SOAR", "TimeUS,nettorate,dx,dy,x0,x1,x2,x3,lat,lng,alt,dx_w,dy_w", "QfffffffLLfff", 
                                               AP_HAL::micros64(),
                                               (double)_vario.reading,
                                               (double)dx,
                                               (double)dy,
                                               (double)_ekf.X[0],
                                               (double)_ekf.X[1],
                                               (double)_ekf.X[2],
                                               (double)_ekf.X[3],
                                               current_loc.lat,
                                               current_loc.lng,
                                               (double)_vario.alt,
                                               (double)dx_w,
                                               (double)dy_w);

        //log_data();
        _ekf.update(_vario.reading,dx, dy);       // update the filter
        _prev_update_location = current_loc;      // save for next time
        _prev_update_time = AP_HAL::micros64();
        _vario.new_data = false;
    }
}

void SoaringController::update_cruising()
{
    // Reserved for future tasks that need to run continuously while in FBWB or AUTO mode,
    // for example, calculation of optimal airspeed and flap angle.
}


void SoaringController::get_heading_estimate(float *hdx, float *hdy) const
{
    Vector2f gnd_vel = _ahrs.groundspeed_vector();
    Vector3f wind = _ahrs.wind_estimate();
    *hdx = gnd_vel.x - wind.x;
    *hdy = gnd_vel.y - wind.y;
}


void SoaringController::get_velocity_estimate(float dt, float *v0) const
{
    float hdx, hdy;
    get_heading_estimate(&hdx, &hdy);
    *v0 = sqrtf(hdx * hdx + hdy * hdy);
}


void SoaringController::update_vario()
{
    _vario.update(polar_K, polar_CD0, polar_B);

    // Fill EKF buffer for eventual action on thermal trigger.
    struct Location current_loc;
    get_position(current_loc);

    float dx = 0;
    float dy = 0;
    float dx_w = 0;
    float dy_w = 0;
    Vector3f wind = _ahrs.wind_estimate();
    get_wind_corrected_drift(&current_loc, &wind, &dx_w, &dy_w, &dx, &dy);

    _ekf_buffer[_ptr][0] = _vario.reading;
    _ekf_buffer[_ptr][1] = dx;
    _ekf_buffer[_ptr][2] = dy;
    _ptr = (_ptr + 1) % EKF_MAX_BUFFER_SIZE;
}


float SoaringController::correct_netto_rate(float climb_rate, float phi, float aspd) const
{
    return _vario.correct_netto_rate(climb_rate, phi, aspd, polar_K, polar_CD0, polar_B);
}

float SoaringController::McCready(float alt)
{
    // A method shell to be filled in later
    return thermal_vspeed;
}

bool SoaringController::is_active() const
{
    if (!soar_active) {
        return false;
    }
    if (soar_active_ch <= 0) {
        // no activation channel
        return true;
    }
    // active when above 1700
    return hal.rcin->read(soar_active_ch-1) >= 1700;
}


bool SoaringController::POMDSoar_active()
{
    return _pomdsoar.are_computations_in_progress();
}

void SoaringController::soaring_policy_computation()
{
    if (pomdp_on)
    {
        _pomdsoar.update_internal_state();
    }
}


void SoaringController::stop_computation()
{
    _pomdsoar.stop_computations();
}


float SoaringController::get_roll_cmd()
{
    return _pomdsoar.get_action();
}

float SoaringController::get_roll() const
{
    return _ahrs.roll;
}


float SoaringController::get_rate() const
{
    return _ahrs.get_gyro().x;
}


void SoaringController::get_position(Location& loc)
{
     _ahrs.get_position(loc);
}


float SoaringController::get_aspd() const
{
    // initialize to an obviously invalid value, which should get overwritten.
    float aspd = -100.0f;

    if (aspd_src == 0)
    {
        if (!_ahrs.airspeed_estimate(&aspd))
        {
            aspd = 0.5f*(_aparm.airspeed_min + _aparm.airspeed_max);
        }
    }
    else if (aspd_src == 1)
    {
        aspd = _wind_corrected_gspd;
    }
    else if (aspd_src == 2)
    {
        if (!_ahrs.airspeed_estimate(&aspd))
        {
            aspd = _aparm.airspeed_cruise_cm / 100.0f;
        }

        aspd -= _wind_ekf.X[0]; // correct sensor aspd for estimated bias
    }

    return aspd;
}


void SoaringController::get_relative_position_wrt_home(Vector2f &vec) const
{
    _ahrs.get_relative_position_NE_home(vec);
}


float SoaringController::get_eas2tas() const
{
    return _ahrs.get_EAS2TAS();
}