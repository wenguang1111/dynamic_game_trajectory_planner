#ifndef UPDATE_TRAJECTORY_INTERFACE_H
#define UPDATE_TRAJECTORY_INTERFACE_H

#include "update_trajectory_ispc.h"

inline void update_trajectory_SIMD(int i,
                                double v_target_speed,
                                const double* U_,
                                double* sr_t0_x,
                                double* sr_t0_y,
                                double* sr_t0_psi,
                                double* ds_t0_v,
                                double* s_t0_s,
                                double* X_,
                                double* s_t0) 
{
    ispc::update_trajectory(i, 
                            v_target_speed,
                            U_, 
                            sr_t0_x,
                            sr_t0_y,
                            sr_t0_psi,
                            ds_t0_v,
                            s_t0_s,
                            X_,
                            s_t0);
}

#endif
