// ispc_parameters.h
#ifndef ISPC_PARAMETERS_H
#define ISPC_PARAMETERS_H

// State indices
enum { x = 0, y = 1, v = 2, psi = 3, s = 4, l = 5 };

// Input indices
enum { d = 0, F = 1 };

// Integration parameters
#define N 20                                  // number of integration nodes
#define nX 6                                  // <X, Y, V, PSI, S, L>
#define nU 2                                  // <d, F>
#define BLOCK_SIZE 4 
#define NUM_State N+1

// Numerical constants
uniform const double tau = 2.0;
uniform const double k = 10.0;
// const double r_safe = 2.5;
// const double r_lane = 3.5;
// const double eps = 1e-6;
uniform const double length = 5.0;
uniform const double cg_ratio = 0.5;
// const double pi = 3.1415;
// const double v_max = 10.0;

// const int N_interpolation = 60;
// const double dt_interpolation = 0.1;

uniform const double dt = 0.3;                             // integration time step
// const double d_up = 0.7;                           // upper bound yaw rate
// const double d_low = -0.7;                         // lower bound yaw rate
// const double F_up = 2.0;                           // upper bound force
// const double F_low = -3.0;                         // lower bound force

// Weights for cost function
uniform const double weight_target_speed = 1e0;            // weight for max speed in lagrangian
uniform const double weight_center_lane = 1e-1;            // weight for center lane
uniform const double weight_heading = 1e2;                 // weight for heading
uniform const double weight_input = 0.0;                   // weight for input

#endif // ISPC_PARAMETERS_H