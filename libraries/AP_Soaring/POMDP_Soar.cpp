// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include <GCS_MAVLink/GCS.h>
#include "AP_Soaring.h"
#include "POMDP_Soar.h"
extern const AP_HAL::HAL& hal;

// ArduSoar parameters
const AP_Param::GroupInfo POMDSoarAlgorithm::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Is the POMDSoar algorithm on?
    // @Description: If 1, the soaring controller uses the POMDSoar algorithm. If 0, the soaring controller uses the ArduSoar algorithm.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, POMDSoarAlgorithm, pomdp_on, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: N
    // @DisplayName: Number of samples per action trajectory used by POMDSoar
    // @Description: Number of samples per action trajectory used by POMDSoar.
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("N", 2, POMDSoarAlgorithm, pomdp_n, 10),

    // @Param: K
    // @DisplayName: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.
    // @Description: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("K", 3, POMDSoarAlgorithm, pomdp_k, 5),

    // @Param: HORI
    // @DisplayName: POMDP planning horizon used by POMDSoar.
    // @Description: POMDP planning horizon used by POMDSoar.
    // @Units: s
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("HORI", 4, POMDSoarAlgorithm, pomdp_hori, 4.0),

    // @Param: STEP
    // @DisplayName: POMDP planning step solve time
    // @Description: The amount of computation time the POMDP solver has for computing the next action
    // @Units: s
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("STEP", 5, POMDSoarAlgorithm, pomdp_step_t, 1),

    // @Param: N_ACT
    // @DisplayName: POMDP number of actions
    // @Description: Number of actions in the POMDP used by POMDSoar. The roll angle input commands corresponding to actions are endpoints of (N_ACT-1) equal intervals between ROLL2 and ROLL1 (inclusive).
    // @Range: 1 254
    // @User: Advanced
    AP_GROUPINFO("N_ACT", 6, POMDSoarAlgorithm, pomdp_n_actions, 2),

    // @Param: PTH
    // @DisplayName: POMDSoar's threshold on tr(P) for switching between explore and max-lift modes.
    // @Description: POMDSoar's threshold on the P matrix trace for switching between explore and max-lift modes.
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("PTH", 7, POMDSoarAlgorithm, pomdp_pth, 50),

    // @Param: EXT
    // @DisplayName: Enable action duration extension in POMDSoar's max-lift mode compared to the explore mode
    // @Description: 0 = off, > 1 = multiplicative factor by which to extend action duration in max-lift compared to the explore mode.
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("EXT", 8, POMDSoarAlgorithm, pomdp_extend, 0),

    // @Param: PLAN
    // @DisplayName: Enable deterministic trajectory planning mode for the POMDP
    // @Description: Enable deterministic trajectory planning mode for the POMDP. 0 = off, 1 on.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("PLAN", 9, POMDSoarAlgorithm, pomdp_plan_mode, 0),

    AP_GROUPEND
};

POMDSoarAlgorithm::POMDSoarAlgorithm(SoaringController *sc, AP_RollController& rollController) :
    _sc(sc),
    _rollController(rollController)
{
    AP_Param::setup_object_defaults(this, var_info);
}


void POMDSoarAlgorithm::init_actions(float roll_limit)
{
    _n_actions = MIN(pomdp_n_actions, MAX_ACTIONS);
    float max_roll = roll_limit;
    float min_roll = 5;

    float roll = min_roll;
    float roll_delta = (max_roll - min_roll) / (_n_actions - 1);
    for (int i = 0; i < _n_actions; i++)
    {
        _roll_cmds[i] = roll;
        roll += roll_delta;
    }
}


void POMDSoarAlgorithm::init_thermalling(float roll_limit)
{
    float heading = _sc->get_yaw();

    init_actions(roll_limit);
    _pomdp_roll_cmd = _roll_cmds[0];

    float aspd    = _sc->get_aspd();
    float eas2tas = _sc->get_eas2tas();
    
    // This assumes that SoaringController called get_position(_prev_update_location) right before this call to init_thermalling
    _sc->get_relative_position_wrt_home(_pomdp_vecNE);

    // Prepare everything necessary for generating action trajectories (by modelling trajectories resulting from various commanded roll angles)
    _solver.set_tau(_rollController.get_gains().tau);
    _solver.set_polar(FUNCTOR_BIND(_sc,  &SoaringController::calculate_aircraft_sinkrate, float, float, float));

    _solver.generate_action_paths(aspd, eas2tas, heading, degrees(_sc->get_roll()),
        _pomdp_roll_cmd, pomdp_k, _n_actions, _roll_cmds,
        pomdp_step_t, pomdp_hori, 0, _pomdp_vecNE);

    _solver.init_step(pomdp_n, _sc->_ekf.X, _sc->_ekf.P, _sc->_ekf.Q, _sc->_ekf.R, _weights, false);
    
    _pomdp_mode        = POMDP_MODE_EXPLORE;
}


float POMDSoarAlgorithm::assess_thermalability()
{
    float aspd = _sc->get_aspd();
    
    float thermalability = -1e6;

    for (int i = 0; i < _n_actions; i++)
    {
        float theta = _roll_cmds[i];
        float expected_thermalling_sink = _sc->calculate_aircraft_sinkrate(radians(theta), aspd);
        float r = (aspd * aspd) / (GRAVITY_MSS * tanf(radians(theta)));
        thermalability = MAX(thermalability, (_sc->_ekf.X[0] * expf(-(r*r) / powf(_sc->_ekf.X[1], 2))) - expected_thermalling_sink);
    }

    return thermalability;
}

void POMDSoarAlgorithm::update_solver()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Thread start");

    while (pomdp_on)
    {
        if (_solver.running() && (AP_HAL::micros64() - _start_time_us) > 100000) {
            _start_time_us = AP_HAL::micros64();
            _solver.update();
            _pomp_loop_time = AP_HAL::micros64() - _start_time_us;
        }
    }
}

float POMDSoarAlgorithm::get_action()
{
    return _pomdp_roll_cmd * 100;
}

void POMDSoarAlgorithm::update_thermalling(const Location &current_loc)
{
    if (!_solver.running() && _pomp_loop_time>0)
    {
        float eas2tas = _sc->get_eas2tas();
        float aspd = _sc->get_aspd();
        uint8_t action = (uint8_t)_solver.get_best_action();
        _pomdp_roll_cmd = _roll_cmds[action];

        gcs().send_text(MAV_SEVERITY_INFO, "Action %f", _pomdp_roll_cmd);

        _sc->get_relative_position_wrt_home(_pomdp_vecNE);
        
        float heading = _sc->get_yaw();

        float n[4] = {
            fabsf(_sc->_ekf.X[0]) > 0.0f ? fabsf(_sc->_ekf.X[0]) : 1.0f,
            fabsf(_sc->_ekf.X[1]) > 0.0f ? fabsf(_sc->_ekf.X[1]) : 1.0f,
            fabsf(_sc->_ekf.X[1]) > 0.0f ? fabsf(_sc->_ekf.X[1]) : 1.0f,
            fabsf(_sc->_ekf.X[1]) > 0.0f ? fabsf(_sc->_ekf.X[1]) : 1.0f
        };

        float trP = _sc->_ekf.P(0, 0) / n[0]
                  + _sc->_ekf.P(1, 1) / n[1]
                  + _sc->_ekf.P(2, 2) / n[2]
                  + _sc->_ekf.P(3, 3) / n[3];

        // Update the correct mode (exploit vs explore) based on EKF covariance trace
        _pomdp_mode = trP < pomdp_pth && pomdp_pth > 0.0f ? POMDP_MODE_EXPLOIT : POMDP_MODE_EXPLORE;

        //  Determine appropriate number of samples.
        //  pompd_n = "Number of samples per action trajectory" - how is this different from n_action_samples?
        int n_samples = pomdp_n;
        float step_w = 1.0f;

        if (_pomdp_mode==POMDP_MODE_EXPLOIT && pomdp_plan_mode)
        {
            // "Enable deterministic trajectory planning mode for the POMDP"
            n_samples = 1;
            step_w = 1.0f / pomdp_n;
        }

        _solver.generate_action_paths(aspd, eas2tas, heading, degrees(_sc->get_roll()), _pomdp_roll_cmd, pomdp_k, _n_actions, _roll_cmds,
            pomdp_step_t * step_w, pomdp_hori, pomdp_extend, _pomdp_vecNE);

        _solver.init_step(n_samples, _sc->_ekf.X, _sc->_ekf.P, _sc->_ekf.Q, _sc->_ekf.R, _weights, _pomdp_mode==POMDP_MODE_EXPLOIT);

        AP::logger().Write("POMP", "TimeUS,loopt,action,x,y,lat,lng,roll,mode,Q", "QQfffLLfBf",
            AP_HAL::micros64(),
            _pomp_loop_time,
            action,
            (double)_pomdp_vecNE.y,
            (double)_pomdp_vecNE.x,
            current_loc.lat,
            current_loc.lng,
            (double)_pomdp_roll_cmd,
            (uint8_t)_pomdp_mode,
            (double)_solver.get_action_Q(action));

        AP::logger().Write("PDBG", "TimeUS,v0,psi,x1,y1,x2,y2,x3,y3,x4,y4", "Qffffffffff",
            AP_HAL::micros64(),
            (double)_solver.get_action_v0(),
            (double)_solver.get_action_path_psi(action, 0),
            (double)_solver.get_action_path_x(action, 1),
            (double)_solver.get_action_path_y(action, 1),
            (double)_solver.get_action_path_x(action, 2),
            (double)_solver.get_action_path_y(action, 2),
            (double)_solver.get_action_path_x(action, 3),
            (double)_solver.get_action_path_y(action, 3),
            (double)_solver.get_action_path_x(action, 4),
            (double)_solver.get_action_path_y(action, 4)
        );
    } else {
        // gcs().send_text(MAV_SEVERITY_INFO, "Solver still running or never ran");
    }
}

bool POMDSoarAlgorithm::planning_init()
{
    if (!pomdp_on) {
        // Must reboot to enable planning thread
        return false;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&POMDSoarAlgorithm::update_solver, void),
                                      "Soaring", 15360, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Could not create soaring stack (%d)", 15360);
        return false;
    }

    return true;
}

bool POMDSoarAlgorithm::healthy()
{
    return pomdp_on && (AP_HAL::micros64() - _start_time_us) < 200000;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

void POMDSoarAlgorithm::update_solver_test()
{
    _solver.update_test();
}

void POMDSoarAlgorithm::run_tests()
{
    int8_t test_id = 1;
    uint64_t start_time = AP_HAL::micros64();
    uint64_t total_time   = 0;
    uint64_t total_time_2 = 0;

    switch (test_id) {
    case 1:
        _solver.run_exp_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring exp test: %lu us", total_time);
        break;
    case 2:
        _solver.run_fast_exp_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fast exp test: %lu us", total_time);
        break;
    case 3:
        _solver.fill_random_array();
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fill rnd array: %lu us", total_time);
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fill rnd array: %lu us", total_time);
        break;
    case 4:
        _solver.run_rnd_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring rnd test: %lu us", total_time);
        break;
    case 5:
        _solver.run_ekf_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring EKF test: %lu us", total_time);
        break;
    case 6:
        _solver.run_loop_test(1000, false);
        total_time = AP_HAL::micros64() - start_time;
        start_time = AP_HAL::micros64();
        _solver.run_loop_test(1000, true);
        total_time_2 = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring loop test: %lu us ML: %lu us", total_time, total_time_2);
        break;
    case 7:
        _solver.run_multivariate_normal_sample_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring multivariate_normal test: %lu us", total_time);
        break;
    case 8:
        // test #8 is run by the SoaringController class itself
        break;
    case 9:
        _solver.run_trig_box_muller_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        start_time = AP_HAL::micros64();
        _solver.run_polar_box_muller_test(1000);
        total_time_2 = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring box-muller trig: %lu us polar: %lu us", total_time, total_time_2);
        break;
    }
}

#endif