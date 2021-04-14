
/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <Filter/AverageFilter.h>
#include "EKF_Polar.h"

static constexpr const uint32_t LEARN_THRESHOLD_TIME = 5000;
static constexpr const float    LEARN_THRESHOLD_ROLL = 0.2f;

class Variometer {

    const AP_Vehicle::FixedWing &_aparm;

    // store time of last update
    uint64_t _prev_update_time;

    float _aspd_filt_constrained;

    float _expected_thermalling_sink;

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    AverageFilterFloat_Size5 _sp_filter;

    /*
     low pass filters for various purposes.
     */
    // Climb rate filter for monitoring progress in thermal.
    LowPassFilter<float> _climb_filter;

    // Fast filter for mavlink/audio vario output.
    LowPassFilter<float> _audio_filter;

    // Slower filter for deciding to enter THERMAL mode.
    LowPassFilter<float> _trigger_filter;

    // Longitudinal acceleration bias filter.
    LowPassFilter<float> _vdotbias_filter;

    // Speed to fly vario filter.
    LowPassFilter<float> _stf_filter;

    EKF_Polar _learn_EKF;
    bool _learn_initialised = false;
    uint32_t _learn_skipped_time   = 0;

public:
    struct PolarParams {
        AP_Float K;
        AP_Float CD0;
        AP_Float B;
    };

    Variometer(const AP_Vehicle::FixedWing &parms, PolarParams &polarParams);

    float alt;
    float reading;
    float tau;

    void update(const float thermal_bank, float exp_e_rate);
    float calculate_aircraft_sinkrate(float phi) const;

    void reset_climb_filter(float value) { _climb_filter.reset(value);}

    void reset_trigger_filter(float value) { _trigger_filter.reset(value);}

    float get_airspeed(void) const {return _aspd_filt_constrained;};

    float get_displayed_value(void) const {return _audio_filter.get();};

    float get_filtered_climb(void) const {return _climb_filter.get();};

    float get_trigger_value(void) const {return _trigger_filter.get();};

    float get_stf_value(void) const {return _stf_filter.get();};

    float get_exp_thermalling_sink(void) const {return _expected_thermalling_sink;};

    float calculate_circling_time_constant(const float thermal_bank);

    void update_polar_learning(bool learn_enabled, bool throttle_suppressed, float dsp_dem);

private:
    PolarParams &_polarParams;

    void reset_polar_learning(void);
};

