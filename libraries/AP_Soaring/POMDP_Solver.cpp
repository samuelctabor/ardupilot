// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include "POMDP_Solver.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#define EKF_FAST_MATH

#ifdef EKF_FAST_MATH
static union
{
    float d;
    int i;
} _eco;
#define EXP_A 12102203 /* int(1<<23/math.log(2)) */
#define EXP_C 0 /* see text for choice of c values */

// Adapted from Schraudolph, "A Fast, Compact Approximation if the Exponential Function",
// Tech Report UDSIA-07-98
#define fastexp(y) (_eco.i = EXP_A*(y)+(1065353216 - EXP_C), _eco.d)
/*
in the above fastexp macro:
values of x around -88 to -89 can result in NaN,
values below about -89 are not valid:
x                                    hex value                                 hex value
-88.0    exp(x) = 6.0546014852e-39   41edc4      fastexp(x) = 5.0357061614e-40     57bc0
-88.5    exp(x) = 3.6723016101e-39   27fce2      fastexp(x) = NaN               ffa92680
-89.0    exp(x) = 2.2273639090e-39   1840fc      fastexp(x) = -2.7225029733e+38 ff4cd180
so we check that x is than 88 to avoid this.
(Note we also assume here that x is always negative, which is the case when used in a gaussian)
*/

#define EXP(x) ( (x) > -88.0f ? fastexp(x) : 0.0 )
#else
#define EXP(x) expf(x)
#endif

#define fastarctan(x) ( M_PI_4*(x) - (x)*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x)) )

PomdpSolver::PomdpSolver()
{
    fill_random_array();
    _i_ptr = 0;
    _s_ptr = 0;
}

void PomdpSolver::generate_action_paths(float v0, float eas2tas, float psi0, float roll0, float current_action, int pomdp_k, int nactions, float* action,
    float t_step, float t_hori, int extend, Vector2f posNE)
{
    // Initialise planning variables.
    _v0 = v0;

    // Determine _k, _t_hori, _t_step
    _n_step = int(t_hori * pomdp_k);
    _t_hori = t_hori;
    _t_step = t_step;

    if (extend > 1)
    {
        _n_step = int(extend * t_hori * pomdp_k);

        if (_n_step > MAX_ACTION_SAMPLES)
        {
            _n_step = MAX_ACTION_SAMPLES;
            _t_hori = _n_step / (float)pomdp_k;
        }
        else
        {
            _t_hori = extend * t_hori;
        }
        _t_step = extend * t_step;
    }

    _actions = action;
    _eas2tas = eas2tas;
    _psi0 = psi0;
    _roll0 = roll0;
    _n_actions = nactions;
    _posNE  = posNE;
}

void PomdpSolver::generate_action(int i_action, float v0, float eas2tas, float psi0, float roll0, float current_action,
    float t_step, float t_hori, Vector2f posNE)
{
    // Plan the possible flight paths. This function will do all the integration steps for the specified action.
    // How much faster do we integrate than EKF updates
    const int rate_x = 100;
    float dt = t_hori / (_n_step * rate_x);

    // Initial variables for a new path.
    float px = posNE.x;
    float py = posNE.y;
    float psi = psi0;
    float theta_cmd = current_action;
    float theta = roll0;
    float t = dt;

    // On first step, initialize path.
    _action_path_x[i_action][0] = px;
    _action_path_y[i_action][0] = py;
    _action_path_psi[i_action][0] = psi;
    _action_path_theta[i_action][0] = theta;

    theta_cmd = _actions[i_action];

    for (int j_step = 0; j_step < _n_step; j_step++)
    {
        // Loop until we reach specified max index, or number of steps per action.
        for (int i = 0; i < rate_x; i++)
        {
            // Perform numerical integration
            float desired_rate = (theta_cmd - theta) / _roll_tau;
            theta += desired_rate * dt;
            psi += dt * (GRAVITY_MSS * tanf((theta * M_PI) / 180.0f) / v0);
            px += dt * v0 * cosf(psi);
            py += dt * v0 * sinf(psi);
            t += dt;
        }

        // Save integrated variables
        _action_path_x[i_action][j_step + 1] = px;
        _action_path_y[i_action][j_step + 1] = py;
        _action_path_psi[i_action][j_step + 1] = psi;
        _action_path_theta[i_action][j_step + 1] = theta;
    }

    if (_r>0.0) {
        log_actions(1);
    }
}

void PomdpSolver::log_actions(uint64_t thermal_id)
{
    float cost;

    for (int m = 0; m < _n_actions; m++)
    {
        for (int j = 0; j < _n_step; j++)
        {
            cost = inner_loop(m,j);
            AP::logger().Write("POMA", "TimeUS,id,m,j,x,y,roll,cost",
                "QQBBffff",
                AP_HAL::micros64(),
                thermal_id,
                m,
                j,
                (double)_action_path_x[m][j],
                (double)_action_path_y[m][j],
                (double)_action_path_theta[m][j],
                (double)cost
            );
        }
    }
}

void PomdpSolver::init_step(int n,
    const VectorN<float, 4> &x0, const MatrixN<float, 4> &p0, const MatrixN<float, 4> &q0, float r0,
    float weights[4], bool max_lift)
{
    _n_sample = n;

    for (int i = 0; i < 4; i++)
    {
        _x0[i] = x0[i];
        _weights[i] = weights[i];	
    }

    _p0 = p0;
    _q0 = q0;
    _r0 = r0;
    cholesky44(_p0.getarray(),_chol_p0);
    _mode_exploit = max_lift;
    _dt = _t_hori / ((float)_n_step);
    _therm_x = _x0[2];
    _therm_y = _x0[3];
    _best_action = 0;
    _Q[0] = 0;
    _running = true;
}

float PomdpSolver::inner_loop(int i_act, int i_step)
{
    // Calculate the total lift and do EKF estimation step for given action and timestep.
    float px1 = _action_path_x[i_act][i_step];
    float py1 = _action_path_y[i_act][i_step];
    float rx = px1 - _x;
    float ry = py1 - _y;
    float z = _w * EXP(-(rx * rx + ry * ry) / (_r * _r));

    float lift = z + _sink_polar(_action_path_theta[i_act][i_step], _v0);
    
    _ekf.update(z, py1, px1, 0.0f, 0.0f);

    return lift;
}

void PomdpSolver::sample_loop()
{
    // Create the random samples
    float s[4];
    if (_n_sample > 1)
    {
        multivariate_normal(s, _mean, _chol_p0);
        //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Sample: %f %f %f %f", s[0], s[1], s[2], s[3]);
        _w = _x0[0] + s[0];
        _r = _x0[1] + s[1];
        _x = _therm_x + s[3]; // Note: state vector index 3 = East = x 
        _y = _therm_y + s[2]; // Note: state vector index 2 = North = y
    }
    else {
        _w = _x0[0];
        _r = _x0[1];
        _x = _therm_x;
        _y = _therm_y;
    }

    _ekf.reset(_x0, _p0, _q0, _r0);
}

void PomdpSolver::update()
{
    // Main numerically intensive function.
    gcs().send_text(MAV_SEVERITY_INFO, "Start solver loop");

    // Generate the actions.
    for (int i_action=0; i_action<_n_actions; i_action++)
    {
        generate_action(i_action, _v0, _eas2tas, _psi0, _roll0, _actions[i_action],
            _t_step, _t_hori, _posNE);
    }
   
    _best_action = 0;

    // Loop over all actions, samples and steps to calculate cost.
    for (int i_action=0; i_action<_n_actions; i_action++) {
        _Q[i_action] = 0;
        float total_lift = 0;

        for (int i_sample=0; i_sample<_n_sample; i_sample++) {
            // sample loop init
            sample_loop();

            for (int i_step=0; i_step<_n_step; i_step++) {
                total_lift += inner_loop(i_action, i_step);
            }

            if (_mode_exploit)
            {
                // maximizing lift = minimizing the negative of the lift
                // this has already been summed over the action steps in inner loop
                _Q[i_action] += - total_lift / _n_sample;;
            }
            else
            {
                // minimising uncertainty - minimising the trace of final EKF covariance
                _Q[i_action] += (_weights[0] * _ekf.P(0,0)
                                + _weights[1] * _ekf.P(1,1)
                                + _weights[2] * _ekf.P(2,2)
                                + _weights[3] * _ekf.P(3,3)) * 1.0 / _n_sample;
            }
       }

        // Cost for this action is now fully computed.
        if (_Q[i_action] < _Q[_best_action])
        {
            _best_action = i_action;
        }
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Solver loop complete, best action %f", _actions[_best_action]);
    _running = false;
}

void PomdpSolver::fill_random_array()
{
    float cov[4][4];
    float mean[4] = { 0, 0, 0, 0 };
    cov[0][0] = 1;
    cov[1][1] = 1;
    cov[2][2] = 1;
    cov[3][3] = 1;
    multivariate_normal_fill(_s, mean, cov, MAX_GAUSS_SAMPLES);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

void PomdpSolver::run_exp_test(unsigned n)
{
    for(unsigned i=0; i < n; i++)
    {
        _dummy[0] = expf(_s[i % MAX_GAUSS_SAMPLES][0]);
        _dummy[1] = expf(_s[i % MAX_GAUSS_SAMPLES][1]);
        _dummy[2] = expf(_s[i % MAX_GAUSS_SAMPLES][2]);
        _dummy[3] = expf(_s[i % MAX_GAUSS_SAMPLES][3]);
    }
}


void PomdpSolver::run_fast_exp_test(unsigned n)
{
    for(unsigned i=0; i < n; i++)
    {
        _dummy[0] = EXP(_s[i % MAX_GAUSS_SAMPLES][0]);
        _dummy[1] = EXP(_s[i % MAX_GAUSS_SAMPLES][1]);
        _dummy[2] = EXP(_s[i % MAX_GAUSS_SAMPLES][2]);
        _dummy[3] = EXP(_s[i % MAX_GAUSS_SAMPLES][3]);
    }
}

void PomdpSolver::run_rnd_test(unsigned n)
{
    for (unsigned i = 0; i < n; i++)
    {
        xorshift128();
    }
}


void PomdpSolver::run_multivariate_normal_sample_test(unsigned n)
{
    float L[4][4];
    float mean[4] = { 0, 0, 0, 0 };
    L[0][0] = 1;
    L[1][1] = 1;
    L[2][2] = 1;
    L[3][3] = 1;
    float s[4];

    for (unsigned i = 0; i < n; i++) {
        multivariate_normal(s,mean,L);
    }
}


void PomdpSolver::run_trig_box_muller_test(unsigned n)
{
    float y1, y2;
    for (unsigned i = 0; i < n; i++) {
        trig_box_muller(&y1,&y2);
    }
}


void PomdpSolver::run_polar_box_muller_test(unsigned n)
{
    float y1, y2;

    for (unsigned i = 0; i < n; i++)
    {
        polar_box_muller(&y1,&y2);
    }
}


void PomdpSolver::run_ekf_test(unsigned n)
{
    VectorN<float, 4> X = (const float[]) {2.5,100,0,0};
    MatrixN<float, 4> P = (const float[]) { 1,100,1000,1000 };
    MatrixN<float, 4> Q = (const float[]) { 0.0025,1,2,2};
    float R = 0.024;
    _ekf.reset(X,P,Q,R);

    for (unsigned i = 0; i < n; i++)
    {
        _ekf.update(0.1, 1.0, 2.0, 0.0, 0.0);
    }
}


void PomdpSolver::run_loop_test(unsigned n, bool max_lift)
{
    VectorN<float, 4> X = (const float[]) { 2.5, 100, 0, 0 };
    MatrixN<float, 4> P = (const float[]) { 1, 100, 1000, 1000 };
    MatrixN<float, 4> Q = (const float[]) { 0.0025, 1, 2, 2 };
    float R = 0.024;
    _ekf.reset(X, P, Q, R);
    _w = X[0];
    _r = X[1];
    _y = X[2];
    _x = X[3];
    _mode_exploit = max_lift;
    int i_action = 0;
    int i_step = 0;
    _action_path_x[i_action][i_step] = 1.0;
    _action_path_y[i_action][i_step] = 2.0;

    for (unsigned i = 0; i < n; i++)
    {
        inner_loop(i_action, i_step);
    }
}


void PomdpSolver::update_random_buffer(unsigned n, MatrixN<float, 4> &cov, bool reset)
{
    unsigned div = MIN(MAX_GAUSS_SAMPLES - _s_ptr, n);
    unsigned rem = n - div;
    float mean[4] = { 0, 0, 0, 0 };
    MatrixN<float, 4> &p = cov;

    if (_running) {
        p = _p0;
    }

    multivariate_normal_fill(_s, mean, p.getarray(), div, _s_ptr);
    
    if (rem > 0) {
        multivariate_normal_fill(_s, mean, p.getarray(), rem, 0);
    }
    
    if (reset) {
        _i_ptr = _s_ptr;
    }

    _s_ptr = (_s_ptr + n) % MAX_GAUSS_SAMPLES;
}

void PomdpSolver::update_test() {
    update_test_counter++;
}


#endif