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
#define BLOCK_SIZE 8 
#define NUM_State 21

// Numerical constants
uniform const float tau = 2.0;
uniform const float k = 10.0;
// const float r_safe = 2.5;
// const float r_lane = 3.5;
// const float eps = 1e-6;
uniform const float length = 5.0;
uniform const float cg_ratio = 0.5;
// const float pi = 3.1415;
// const float v_max = 10.0;

// const int N_interpolation = 60;
// const float dt_interpolation = 0.1;

uniform const float dt = 0.3;                             // integration time step
// const float d_up = 0.7;                           // upper bound yaw rate
// const float d_low = -0.7;                         // lower bound yaw rate
// const float F_up = 2.0;                           // upper bound force
// const float F_low = -3.0;                         // lower bound force

// Weights for cost function
uniform const float weight_target_speed = 1e0;            // weight for max speed in lagrangian
uniform const float weight_center_lane = 1e-1;            // weight for center lane
uniform const float weight_heading = 1e2;                 // weight for heading
uniform const float weight_input = 0.0;                   // weight for input

#endif // ISPC_PARAMETERS_H