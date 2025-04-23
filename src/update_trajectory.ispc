#include "ispc_parameter.h"

export void update_trajectory(uniform int i,
                               uniform double v_target_speed,
                               const uniform double U_[],
                               uniform double sr_t0_x[],
                               uniform double sr_t0_y[],
                               uniform double sr_t0_psi[],
                               uniform double ds_t0_v[],
                               uniform double s_t0_s[],
                               uniform double X_[],
                               uniform double init_s_t0[]) 
{
    foreach (j = 0 ... N) {
        int tu = nU * N * i + nU * j;
        int td = nX * N * i + nX * j;
        double u_t0[nU];
        double ds_t0[nX];
        double sr_t0[nX];
        double s_t0[nX];
        s_t0[x] = init_s_t0[x];
        s_t0[y] = init_s_t0[y];
        s_t0[v] = init_s_t0[v];
        s_t0[psi] = init_s_t0[psi];
        s_t0[s] = init_s_t0[s];
        s_t0[l] = init_s_t0[l];

        // Reference point
        sr_t0[x] = sr_t0_x[j];
        sr_t0[y] = sr_t0_y[j];
        sr_t0[psi] = sr_t0_psi[j];

        // Target speed
        double v_ref = v_target_speed;

        // Input control
        u_t0[d] = U_[tu + d];
        u_t0[F] = U_[tu + F];

        // Derivatives
        ds_t0[x] = s_t0[v] * cos(s_t0[psi] + cg_ratio * u_t0[d]);
        ds_t0[y] = s_t0[v] * sin(s_t0[psi] + cg_ratio * u_t0[d]);
        ds_t0[v] = ds_t0_v[j];
        ds_t0[psi] = s_t0[v] * tan(u_t0[d]) * cos(cg_ratio * u_t0[d]) / length;

        ds_t0[l] = weight_target_speed * (s_t0[v] - v_ref) * (s_t0[v] - v_ref)
                 + weight_center_lane * ((sr_t0[x] - s_t0[x]) * (sr_t0[x] - s_t0[x]) 
                 + (sr_t0[y] - s_t0[y]) * (sr_t0[y] - s_t0[y]))
                 + weight_heading * ((cos(sr_t0[psi]) - cos(s_t0[psi])) * (cos(sr_t0[psi]) - cos(s_t0[psi]))
                 + (sin(sr_t0[psi]) - sin(s_t0[psi])) * (sin(sr_t0[psi]) - sin(s_t0[psi])))
                 + weight_input * u_t0[F] * u_t0[F];

        // Integration
        s_t0[x] += dt * ds_t0[x];
        s_t0[y] += dt * ds_t0[y];
        s_t0[v] = ds_t0_v[j];
        s_t0[psi] += dt * ds_t0[psi];
        s_t0[s] = s_t0_s[j];
        s_t0[l] += dt * ds_t0[l];

        // Save the state
        X_[td + x] = s_t0[x];
        X_[td + y] = s_t0[y];
        X_[td + v] = s_t0[v];
        X_[td + psi] = s_t0[psi];
        X_[td + s] = s_t0[s];
        X_[td + l] = s_t0[l];
    }
}