// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#pragma once

#include <APM_Control/APM_Control.h>
#include "POMDP_Solver.h"
class SoaringController;

enum POMDP_Mode
{
    POMDP_MODE_EXPLORE = 0,
    POMDP_MODE_EXPLOIT = 1
};

//
// POMDSoarAlgorithm, the POMDP/Bayesian RL-based logic used by SoaringController to decide on the course of action
//
class POMDSoarAlgorithm
{
    friend class SoaringController;

private:
    float _roll_cmds[MAX_ACTIONS];
    int _n_actions = 2;
    uint64_t _pomp_loop_time;
    float _pomdp_roll_cmd = 0;
    Vector2f _pomdp_vecNE;
    POMDP_Mode _pomdp_mode = POMDP_MODE_EXPLORE;
    float _weights[4] = { 1, 1, 1, 1 };

    AP_RollController& _rollController;
    PomdpSolver _solver;
    SoaringController *_sc;

    uint64_t _start_time_us;

    AP_Int8 pomdp_on;
    AP_Int16 pomdp_n;
    AP_Int16 pomdp_k;
    AP_Float pomdp_hori;
    AP_Float pomdp_step_t;
    AP_Float pomdp_loop_load;
    AP_Int8 pomdp_n_actions;
    AP_Int8 pomdp_extend;
    AP_Int8 pomdp_plan_mode;
    AP_Float pomdp_pth;

    void init_actions(float roll_limit);

public:
    POMDSoarAlgorithm(SoaringController *sc, AP_RollController& rollController);
    static const struct AP_Param::GroupInfo var_info[];
    void init_thermalling(float roll_limit);
    float assess_thermalability();
    void update_solver();
    void update_solver_test();
    float get_action();
    void update_thermalling(const Location &current_loc);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    void run_tests();
#endif
    bool planning_init();
    bool healthy();

};
