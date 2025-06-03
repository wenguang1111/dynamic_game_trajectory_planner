#include "ispc_parameter.h"
// #include <stdio.h>

struct State_Block{
    double x[BLOCK_SIZE];
    double y[BLOCK_SIZE];
    double v[BLOCK_SIZE];
    double psi[BLOCK_SIZE];
    double v_target[BLOCK_SIZE];
};

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
    Block_X s_t0;
    double ds_t0[nX];
    double u_t0[nX];

    for(uniform int i = 0; i<num_block; i++){
        foreach(q=0 ... BLOCK_SIZE)
        {
            s_t0.x[q] = states[i].x[q];
            s_t0.y[q] = states[i].y[q];
            s_t0.v[q]= states[i].v[q];
            s_t0.psi[q] = states[i].psi[q];
            s_t0.s[q] = 0.0;
        }
        for (uniform int j = 0; j < NUM_State; j++){
            uniform int index = i * NUM_State + j;
            foreach(q = 0 ... BLOCK_SIZE)
            {
                s_t0_out[index].x[q] = s_t0.x[q];
                s_t0_out[index].y[q] = s_t0.y[q];
                s_t0_out[index].v[q] = s_t0.v[q];
                s_t0_out[index].psi[q] = s_t0.psi[q];
                s_t0_out[index].s[q] = s_t0.s[q];
                
                u_t0[d] = U_[index].d[q];
                u_t0[F] = U_[index].F[q];
                
                // Derivatives: 
                ds_t0[x] = s_t0.v[q] * cos(s_t0.psi[q] + cg_ratio * u_t0[d]);
                ds_t0[y] = s_t0.v[q] * sin(s_t0.psi[q] + cg_ratio * u_t0[d]);
                ds_t0[v] = (-1/tau) * s_t0.v[q] + (k) * u_t0[F];
                ds_t0[psi] = s_t0.v[q] * tan(u_t0[d]) * cos(cg_ratio * u_t0[d])/ length;
                ds_t0[s] = s_t0.v[q];

                s_t0.x[q] += dt * ds_t0[x];
                s_t0.y[q] += dt * ds_t0[y];
                s_t0.v[q] += dt * ds_t0[v];
                s_t0.psi[q] += dt * ds_t0[psi];
                s_t0.s[q] += dt * ds_t0[s];

                if (s_t0.v[q] < 0.0){s_t0.v[q] = 0.0;}

                X_[index].x[q] = s_t0.x[q];
                X_[index].y[q] = s_t0.y[q];
                X_[index].v[q] = s_t0.v[q];
                X_[index].psi[q] = s_t0.psi[q];
                X_[index].s[q] = s_t0.s[q];
            }
        }
    }
}