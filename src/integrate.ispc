#include "ispc_parameter.h"
// #include <stdio.h>

struct State_Block{
    float x[BLOCK_SIZE];
    float y[BLOCK_SIZE];
    float v[BLOCK_SIZE];
    float psi[BLOCK_SIZE];
    float v_target[BLOCK_SIZE];
};

struct Block_U
{
    float d[BLOCK_SIZE];
    float F[BLOCK_SIZE];
};

struct Block_X
{
    float x[BLOCK_SIZE];
    float y[BLOCK_SIZE];
    float psi[BLOCK_SIZE];
    float v[BLOCK_SIZE];
    float s[BLOCK_SIZE];
    float l[BLOCK_SIZE];
};


export void __keep_Lanes_ISPC_alive(uniform Block_U& sa, uniform Block_X& das) {
    Block_U dummy_block;
    Block_X dummy_block_x;
}

/** integrates the input U to get the state X */
export void integrate_ispc(uniform Block_X X_[], uniform const Block_U U_[], uniform Block_X s_t0_out[], uniform const State_Block states[],
                            uniform const int num_block)
{
    uniform Block_X s_t0;
    uniform Block_X ds_t0;
    uniform Block_U u_t0;

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
                
                u_t0.d[q] = U_[index].d[q];
                u_t0.F[q] = U_[index].F[q];
                
                // Derivatives:
                float a = cg_ratio * u_t0.d[q];
                float b = s_t0.psi[q] + a; 
                ds_t0.x[q] = s_t0.v[q] * cos(b);
                ds_t0.y[q] = s_t0.v[q] * sin(b);
                ds_t0.v[q] = (-1/tau) * s_t0.v[q] + (k) * u_t0.F[q];
                ds_t0.psi[q] = s_t0.v[q] * tan(u_t0.d[q]) * cos(a)/ length;
                ds_t0.s[q] = s_t0.v[q];

                s_t0.x[q] += dt * ds_t0.x[q];
                s_t0.y[q] += dt * ds_t0.y[q];
                s_t0.v[q] += dt * ds_t0.v[q];
                s_t0.psi[q] += dt * ds_t0.psi[q];
                s_t0.s[q] += dt * ds_t0.s[q];

                // if (s_t0.v[q] < 0.0){s_t0.v[q] = 0.0;}
                s_t0.v[q] = max(0.0f, s_t0.v[q]);

                X_[index].x[q] = s_t0.x[q];
                X_[index].y[q] = s_t0.y[q];
                X_[index].v[q] = s_t0.v[q];
                X_[index].psi[q] = s_t0.psi[q];
                X_[index].s[q] = s_t0.s[q];
            }
        }
    }
}