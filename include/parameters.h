#ifndef PARAMETERS_H
#define PARAMETERS_H

enum STATES {x, y, v, psi, s, l};
enum INPUTS {d, F};

#define BLOCK_SIZE 4 
#define NUM_State 21
struct Parameters
{
    const double tau = 2.0;
    const double k = 10.0;
    const double r_safe = 2.5;
    const double r_lane = 3.5;
    const double eps = 1e-6;
    const double length = 5.0;
    const double cg_ratio = 0.5;
    const double pi = 3.1415;
    const double v_max = 10.0;

    constexpr static const int N = 20;                                  /** number of integration nodes */
    constexpr static const int nX = 6;                                  /** <X, Y, V, PSI, S, L> */
    constexpr static const int nU = 2;                                  /** <d, F> */
    constexpr static const int N_interpolation = 60;
    constexpr static const double dt_interpolation = 0.1;

    const double dt = 0.3;                                                    /** integration time step */
    const double d_up = 0.7;                                                  /** upper bound yaw rate */
    const double d_low = -0.7;                                                /** lower bound yaw rate */
    const double F_up = 2.0;                                                  /** upper bound force */
    const double F_low = -3.0;                                                /** lower bound force */

    // Parameters:
    const double weight_target_speed = 1e0;                                      /** weight for the maximum speed in the lagrangian */
    const double weight_center_lane = 1e-1;                                   /** weight for the center lane in the lagrangian */
    const double weight_heading = 1e2;                                        /** weight for the heading in the lagrangian */
    const double weight_input = 0.0;                                          /** weight for the input in the lagrangian */
};

#endif // PARAMETERS_H