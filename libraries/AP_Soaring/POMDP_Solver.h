// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include "ExtendedKalmanFilter.h"
#include "random.h"
#include <AP_Math/matrixN.h>
#include <AP_Math/AP_Math.h>


#define MAX_ACTIONS 10
#define MAX_ACTION_SAMPLES 100
#define ACTION_GENERATION_STEPS_PER_LOOP 1000
class PomdpSolver
{

public:
    FUNCTOR_TYPEDEF(aircraft_sinkrate_fn_t, float, float, float);

private:
    struct {
        float airspeed_min;
    } _aparm;

    float _roll_tau;

    ExtendedKalmanFilter _ekf;
    float _Q[MAX_ACTIONS]; // quality of action
    float _s[MAX_GAUSS_SAMPLES][4]; // Gaussian samples
    unsigned _s_ptr = 0;
    unsigned _i_ptr = 0;
    float _action_path_x[MAX_ACTIONS][MAX_ACTION_SAMPLES + 1];
    float _action_path_y[MAX_ACTIONS][MAX_ACTION_SAMPLES + 1];
    float _action_path_psi[MAX_ACTIONS][MAX_ACTION_SAMPLES + 1];
    float _action_path_theta[MAX_ACTIONS][MAX_ACTION_SAMPLES + 1];
    int _n_sample; // number of samples per action trajectory
    int _n_step; // number of action steps (_i_step)
    int _n_actions; // numer of actions (_i_action)
    float _action[MAX_ACTIONS];
    float _t_step;
    float _t_hori;
    VectorN<float, 4> _x0;
    MatrixN<float, 4> _p0;
    MatrixN<float, 4> _q0;
    float _r0;
    float _chol_p0[4][4];
    float _weights[4];
    float _mean[4] = { 0, 0, 0, 0 };
    bool _mode_exploit;
    float _dt;
    float _therm_x;
    float _therm_y;
    int _best_action;
    float _v0;
    float _w ;
    float _r ;
    float _x ;
    float _y ;
    bool _running = false;
    float inner_loop(int i_act, int i_step);;
    void sample_loop();
    float _dummy[4];
    float _eas2tas;
    float _psi0;
    float _roll0;
    float* _actions;
    Vector2f _posNE;
    aircraft_sinkrate_fn_t _sink_polar;
    

public:
    PomdpSolver(); 
    void set_tau(float tau) {_roll_tau = tau;};
    void set_polar(aircraft_sinkrate_fn_t sink_polar) {_sink_polar = sink_polar;};
    void generate_action_paths(float v0, float eas2tas, float psi0, float roll0, float current_action, int pomdp_k, int nactions, float* action,
        float t_step, float t_hori, int extend, Vector2f posNE);
    void generate_action(int i_action, float v0, float eas2tas, float psi0, float roll0, float current_action,
        float t_step, float t_hori, Vector2f posNE);
    void log_actions(uint64_t thermal_id);
    void init_step(int n,const VectorN<float, 4> &x0, 
                   const MatrixN<float, 4> &p0, const MatrixN<float, 4> &q0, float r0,
                   float weights[4], bool max_lift);
    void update();
    int get_best_action() { return _best_action;  }
    float get_action_path_x(int a, int k) { return _action_path_x[a][k]; }
    float get_action_path_y(int a, int k) { return _action_path_y[a][k]; }
    float get_action_path_theta(int a, int k) { return _action_path_theta[a][k]; }
    float get_action_path_psi(int a, int k) { return _action_path_psi[a][k]; }
    float get_action_v0() { return _v0; }
    float get_action_Q(int i) { return _Q[i]; }

    bool running() { return _running;  }
    void run_exp_test(unsigned n);
    void run_fast_exp_test(unsigned n);
    void fill_random_array();
    void run_rnd_test(unsigned n);
    void run_multivariate_normal_sample_test(unsigned n);
    void run_trig_box_muller_test(unsigned n);
    void run_polar_box_muller_test(unsigned n);
    void run_ekf_test(unsigned n);
    void run_loop_test(unsigned n, bool max_lift);
    void update_random_buffer(unsigned n, MatrixN<float, 4> &cov, bool reset);
    void update_test();
    unsigned update_test_counter = 0;
};