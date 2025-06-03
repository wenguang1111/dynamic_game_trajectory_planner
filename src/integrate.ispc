#include "ispc_parameter.h"
// #include <stdio.h>


# define M_PI		3.14159265358979323846	/* pi */

// struct Spline_ISPC
// {
//     const double* m_x[BLOCK_SIZE];
//     const double* m_y[BLOCK_SIZE];
//     const double* m_b[BLOCK_SIZE];
//     const double* m_c[BLOCK_SIZE];
//     const double* m_d[BLOCK_SIZE];
//     const double m_c0[BLOCK_SIZE];
//     const size_t _size;
// };

// struct Lanes_ISPC{
//     Spline_ISPC spline_x;
//     Spline_ISPC spline_y;
// };

struct State_Block{
    double x[BLOCK_SIZE];
    double y[BLOCK_SIZE];
    double v[BLOCK_SIZE];
    double psi[BLOCK_SIZE];
    double v_target[BLOCK_SIZE];
};

// struct VehicleState_SIMD
// {
//     double x;
//     double y;
//     double psi;
//     double v;
//     double s;
//     double l;
// };

// struct VehicleState_Block
// {
//     double x[BLOCK_SIZE];
//     double y[BLOCK_SIZE];
//     double v[BLOCK_SIZE];
//     double psi[BLOCK_SIZE];
//     double s[BLOCK_SIZE];
// };

// struct PhysicState_SIMD
// {
//     double d;
//     double F;
// };

struct Block_U
{
    double d[BLOCK_SIZE];
    double F[BLOCK_SIZE];
};

struct Block_X
{
    double x[BLOCK_SIZE];
    double y[BLOCK_SIZE];
    double psi[BLOCK_SIZE];
    double v[BLOCK_SIZE];
    double s[BLOCK_SIZE];
    double l[BLOCK_SIZE];
};


export void __keep_Lanes_ISPC_alive(uniform Block_U& sa, uniform Block_X& das) {
    Block_U dummy_block;
    Block_X dummy_block_x;
}

/** integrates the input U to get the state X */
export void integrate_ispc(uniform Block_X X_[], uniform const Block_U U_[], uniform Block_X s_t0_out[], uniform const State_Block states[],
                            uniform const int num_block)
{
    int tu;
    int td;
    Block_X* s_t0 = new Block_X[num_block];
    Block_U* u_t0 = new Block_U[num_block];
    Block_X* ds_t0 = new Block_X[num_block];
    uniform int nx = nX * (N + 1);
    for(uniform int i = 0; i<num_block; i++){
        foreach(j = 0 ... BLOCK_SIZE){
            s_t0[i].x[j] = states[i].x[j];
            s_t0[i].y[j] = states[i].y[j];
            s_t0[i].v[j]= states[i].v[j];
            s_t0[i].psi[j] = states[i].psi[j];
            s_t0[i].s[j] = 0.0;
        }
    }
    for(uniform int i = 0; i<num_block; i++){
        for (uniform int j = 0; j < NUM_State; j++){
            uniform int index = i * NUM_State + j;
            foreach (q = 0 ... BLOCK_SIZE)
            {
                // Reference point on the center lane:
                s_t0_out[index].x[q] = s_t0[i].x[q];
                s_t0_out[index].y[q] = s_t0[i].y[q];
                s_t0_out[index].v[q] = s_t0[i].v[q];
                s_t0_out[index].psi[q] = s_t0[i].psi[q];
                s_t0_out[index].s[q] = s_t0[i].s[q];

                // Input control:
                u_t0[i].d[q] = U_[index].d[q];
                u_t0[i].F[q] = U_[index].F[q];
                
                // Derivatives: 
                ds_t0[i].x[q] = s_t0[i].v[q] * cos(s_t0[i].psi[q] + cg_ratio * u_t0[i].d[q]);
                ds_t0[i].y[q] = s_t0[i].v[q] * sin(s_t0[i].psi[q] + cg_ratio * u_t0[i].d[q]);
                ds_t0[i].v[q] = (-1/tau) * s_t0[i].v[q] + (k) * u_t0[i].F[q];
                ds_t0[i].psi[q] = s_t0[i].v[q] * tan(u_t0[i].d[q]) * cos(cg_ratio * u_t0[i].d[q])/ length;
                ds_t0[i].s[q] = s_t0[i].v[q];

                // Integration to compute the new state: 
                s_t0[i].x[q] += dt * ds_t0[i].x[q];
                s_t0[i].y[q] += dt * ds_t0[i].y[q];
                s_t0[i].v[q] += dt * ds_t0[i].v[q];
                s_t0[i].psi[q] += dt * ds_t0[i].psi[q];
                s_t0[i].s[q] += dt * ds_t0[i].s[q];

                if (s_t0[i].v[q] < 0.0){s_t0[i].v[q] = 0.0;}

                // Save the state in the trajectory
                X_[index].x[q] = s_t0[i].x[q];
                X_[index].y[q] = s_t0[i].y[q];
                X_[index].v[q] = s_t0[i].v[q];
                X_[index].psi[q] = s_t0[i].psi[q];
                X_[index].s[q] = s_t0[i].s[q]; 
            }
        }
    }
    delete[] s_t0;
    delete[] ds_t0;
}