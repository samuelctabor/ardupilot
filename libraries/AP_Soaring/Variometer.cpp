/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#include "Variometer.h"

#include <AP_Logger/AP_Logger.h>

Variometer::Variometer(const AP_Vehicle::FixedWing &parms, PolarParams &polarParams) :
    _aparm(parms),
    _polarParams(polarParams)
{
    _climb_filter    = LowPassFilter<float>(1.0f/60.0f);
    _audio_filter    = LowPassFilter<float>(1.0f/0.71f);
    _trigger_filter  = LowPassFilter<float>(1.0f/6.0f);
    _vdotbias_filter = LowPassFilter<float>(1.0f/60.0f);
    _stf_filter      = LowPassFilter<float>(1.0f/20.0f);
}

void Variometer::update(const float thermal_bank, float exp_e_rate)
{
    const AP_AHRS &_ahrs = AP::ahrs();

    _ahrs.get_relative_position_D_home(alt);
    alt = -alt;

    float aspd = 0;
    if (!_ahrs.airspeed_estimate(aspd)) {
            aspd = _aparm.airspeed_cruise_cm / 100.0f;
    }

    float aspd_filt = _sp_filter.apply(aspd);

    // Constrained airspeed.
    const float minV = sqrtf(_polarParams.K/1.5);
    _aspd_filt_constrained = aspd_filt>minV ? aspd_filt : minV;

    tau = calculate_circling_time_constant(radians(thermal_bank));

    float dt = (float)(AP_HAL::micros64() - _prev_update_time)/1e6;

    // Logic borrowed from AP_TECS.cpp
    // Update and average speed rate of change
    // Get DCM
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Calculate speed rate of change
    float temp = rotMat.c.x * GRAVITY_MSS + AP::ins().get_accel().x;
    // take 5 point moving average
    float dsp = _vdot_filter.apply(temp);

    // Now we need to high-pass this signal to remove bias.
    _vdotbias_filter.set_cutoff_frequency(1.0f/(20.0f*tau));
    float dsp_bias = _vdotbias_filter.apply(temp, dt);
    
    float dsp_cor = dsp - dsp_bias;

    _stf_filter.set_cutoff_frequency(1.0f/tau);
    _trigger_filter.set_cutoff_frequency(2.0f/tau);


    Vector3f velned;

    _raw_climb_rate = 0.0f;
    if (_ahrs.get_velocity_NED(velned)) {
        // if possible use the EKF vertical velocity
        _raw_climb_rate = -velned.z;
    }
    
    _climb_filter.set_cutoff_frequency(1.0f/(3.0f*tau));
    float smoothed_climb_rate = _climb_filter.apply(_raw_climb_rate, dt);

    // Compute still-air sinkrate
    float roll = _ahrs.roll;
    float sinkrate = calculate_aircraft_sinkrate(roll);

    // Add contribution from throttle
    float thr_climb = exp_e_rate/GRAVITY_MSS;

    reading = _raw_climb_rate + dsp_cor*_aspd_filt_constrained/GRAVITY_MSS + sinkrate - thr_climb;
    
    // Update filters.

    float filtered_reading = _trigger_filter.apply(reading, dt);

    _audio_filter.apply(reading, dt);

    _stf_filter.apply(reading, dt);

    _prev_update_time = AP_HAL::micros64();

    _expected_thermalling_sink = calculate_aircraft_sinkrate(radians(thermal_bank));

// @LoggerMessage: VAR
// @Vehicles: Plane
// @Description: Variometer data
// @Field: TimeUS: Time since system startup
// @Field: aspd_filt: filtered and constrained airspeed
// @Field: alt: AHRS altitude
// @Field: roll: AHRS roll
// @Field: raw: estimated air vertical speed
// @Field: filt: low-pass filtered air vertical speed
// @Field: cl: raw climb rate
// @Field: fc: filtered climb rate
// @Field: exs: expected sink rate relative to air in thermalling turn
// @Field: dsp: average acceleration along X axis
// @Field: dspb: detected bias in average acceleration along X axis
// @Field: sr: calculated sinkrate from glide polar params
// @Field: tcl: expected climb rate from current throttle
    AP::logger().Write("VAR", "TimeUS,aspd_filt,alt,roll,raw,filt,cl,fc,exs,dsp,dspb,sr,tcl", "Qffffffffffff",
                       AP_HAL::micros64(),
                       (double)_aspd_filt_constrained,
                       (double)alt,
                       (double)roll,
                       (double)reading,
                       (double)filtered_reading,
                       (double)_raw_climb_rate,
                       (double)smoothed_climb_rate,
                       (double)_expected_thermalling_sink,
                       (double)dsp,
                       (double)dsp_bias,
                       (double)sinkrate,
                       (double)thr_climb);
}


float Variometer::calculate_aircraft_sinkrate(float phi) const
{
    // Remove aircraft sink rate
    float CL0;  // CL0 = 2*W/(rho*S*V^2)
    float C1;   // C1 = CD0/CL0
    float C2;   // C2 = CDi0/CL0 = B*CL0
    CL0 = _polarParams.K / (_aspd_filt_constrained * _aspd_filt_constrained);

    C1 = _polarParams.CD0 / CL0;  // constant describing expected angle to overcome zero-lift drag
    C2 = _polarParams.B * CL0;    // constant describing expected angle to overcome lift induced drag at zero bank

    float cosphi = (1 - phi * phi / 2); // first two terms of mclaurin series for cos(phi)
    
    return _aspd_filt_constrained * (C1 + C2 / (cosphi * cosphi));
}

float Variometer::calculate_circling_time_constant(float thermal_bank)
{
    // Calculate a time constant to use to filter quantities over a full thermal orbit.
    // This is used for rejecting variation in e.g. climb rate, or estimated climb rate
    // potential, as the aircraft orbits the thermal.
    // Use the time to circle - variations at the circling frequency then have a gain of 25%
    // and the response to a step input will reach 64% of final value in three orbits.
    return 2*M_PI*_aspd_filt_constrained/(GRAVITY_MSS*tanf(thermal_bank));
}


void Variometer::reset_polar_learning()
{
// Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode

    const float init_q[2] = {0,
                             0};

    const MatrixN<float,2> q{init_q};

    const float init_p[2] = {powf(0.005,2),
                             powf(0.005,2)};

    const MatrixN<float,2> p{init_p};


    // New state vector filter will be reset. Thermal location is placed in front of a/c
    const float init_xr[2] = {_polarParams.CD0,
                              _polarParams.B};

    const VectorN<float,2> xr{init_xr};

    // Also reset covariance matrix p so filter is not affected by previous data
    _learn_EKF.reset(xr, p, q, powf(0.5,2));
}

void Variometer::update_polar_learning(bool learn_enabled, bool throttle_suppressed, float dsp_dem)
{
    // If learning disabled, mark for filter intitialisation.
    if (!learn_enabled) {
        _learn_initialised = false;
        return;
    }

    // Check if conditions are suitable for updating the glide polar estimator.
    // Conditioned on throttle, speed, roll, rate of change of speed target
    float roll = AP::ahrs().roll;



    if (!throttle_suppressed ||
            abs(roll) > LEARN_THRESHOLD_ROLL ||
            _aspd_filt_constrained < 0.7*_aparm.airspeed_min ||
            _aspd_filt_constrained > 1.3*_aparm.airspeed_max ||
            abs(dsp_dem) > 0.2) {
        // Conditions not ok.
        _learn_skipped_time = AP_HAL::millis();

    } else if ((AP_HAL::millis() - _learn_skipped_time) > LEARN_THRESHOLD_TIME) {

        // Condiditions OK
        if (!_learn_initialised) {
            reset_polar_learning();
            _learn_initialised = true;
        }

        const float u[3] = {_aspd_filt_constrained, roll, _polarParams.K};
        const VectorN<float,3> u_in{u};

        // update the filter
        float input = -_raw_climb_rate;
        _learn_EKF.update(input, u_in);

        // Save parameters if changed by >0.1%
        if (abs((_polarParams.CD0 - _learn_EKF.X[0])/_polarParams.CD0) > 0.001f ||
            abs((_polarParams.B   - _learn_EKF.X[1])/_polarParams.B)   > 0.001f) {
            _polarParams.CD0.set(_learn_EKF.X[0]);
            _polarParams.CD0.save();
            AP::logger().Write_Parameter("SOAR_POLAR_CD0",_learn_EKF.X[0]);

            _polarParams.B.set(_learn_EKF.X[1]);
            _polarParams.B.save();
            AP::logger().Write_Parameter("SOAR_POLAR_B",_learn_EKF.X[1]);
        }

        // Log data
        AP::logger().Write("PLRN", "TimeUS,aspd,roll,K,z,CD0,B", "Qffffff",
                   AP_HAL::micros64(),
                   (double)_aspd_filt_constrained,
                   (double)roll,
                   (double)_polarParams.K,
                   (double)_raw_climb_rate,
                   (double)_learn_EKF.X[0],
                   (double)_learn_EKF.X[1]);
    }
}
