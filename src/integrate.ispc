#include "ispc_parameter.h"
// #include <stdio.h>


# define M_PI		3.14159265358979323846	/* pi */

struct Spline_ISPC
{
    const double* m_x[MAX_OBSTACLES];
    const double* m_y[MAX_OBSTACLES];
    const double* m_b[MAX_OBSTACLES];
    const double* m_c[MAX_OBSTACLES];
    const double* m_d[MAX_OBSTACLES];
    const double m_c0[MAX_OBSTACLES];
    const size_t _size;
};

struct Lanes_ISPC{
    Spline_ISPC spline_x;
    Spline_ISPC spline_y;
};

struct State_ISPC{
    double x[MAX_OBSTACLES];
    double y[MAX_OBSTACLES];
    double v[MAX_OBSTACLES];
    double psi[MAX_OBSTACLES];
    double v_target[MAX_OBSTACLES];
};

struct VehicleState_SIMD
{
    double x;
    double y;
    double v;
    double psi;
    double s;
    double l;
};

struct VehicleState_SOA_SIMD
{
    double x[MAX_OBSTACLES*(N + 1)];
    double y[MAX_OBSTACLES*(N + 1)];
    double v[MAX_OBSTACLES*(N + 1)];
    double psi[MAX_OBSTACLES*(N + 1)];
    double s[MAX_OBSTACLES*(N + 1)];
};

struct PhysicState_SIMD
{
    double d;
    double F;
};

export void __keep_Lanes_ISPC_alive(uniform Lanes_ISPC& dummy) {
    Lanes_ISPC dummy2;
}

// //TODO: check correctness of the following function
// int linear_upper_bound_v(const uniform double* arr,
//                                         uniform int size,
//                                         uniform double value) {
//     varying int result = size; // default: not found
//     foreach (i = 0 ... size) {
//         // mask for which lanes have not yet found a match AND arr[i] > value
//         varying bool m = (result == size) & (arr[i] > value);
//         result = select(m, i, result);
//     }
//     return result;
// }

// int upper_bound(uniform const double* arr, int size, double value) {
//     int low = 0, high = size;
//     while (low < high) {
//         int mid = (low + high) >> 1;
//         if (arr[mid] <= value)
//             low = mid + 1;
//         else
//             high = mid;
//     }
//     return low;
// }

// size_t find_closest(double target_x, uniform Spline_ISPC* varying spline, size_t size)
// {
//     int it = upper_bound(spline->m_x, size, target_x);       // *it > x
//     size_t idx = max((int)(it-spline->m_x[0]-1), 0);   // m_x[idx] <= x
//     return idx;
// }

// double calculate_position(double s_ref, uniform Spline_ISPC* spline)
// {
//     size_t n=spline->_size;
//     size_t idx=find_closest(s_ref,spline, n);

//     double h = s_ref - spline->m_x[idx];
//     double interpol;
//     if(s_ref<spline->m_x[0]) {
//         // extrapolation to the left
//         interpol=(*(spline->m_c0)*h + spline->m_b[0])*h + spline->m_y[0];
//     } else if(s_ref > spline->m_x[n-1]) {
//         // extrapolation to the right
//         interpol=(spline->m_c[n-1]*h + spline->m_b[n-1])*h + spline->m_y[n-1];
//     } else {
//         // interpolation
//         interpol=((spline->m_d[idx]*h + spline->m_c[idx])*h + spline->m_b[idx])*h + spline->m_y[idx];
//     }
//     return interpol;
// }

// double deriv(int order, double x_position, uniform Spline_ISPC* spline)
// {
//     //assert(order>0);
//     size_t n = spline->_size;
//     size_t idx = find_closest(x_position, spline, n);

//     double h=x_position-spline->m_x[idx];
//     double interpol;
//     if(x_position<spline->m_x[0]) {
//         // extrapolation to the left
//         switch(order) {
//         case 1:
//             interpol=2.0*(*(spline->m_c0))*h + spline->m_b[0];
//             break;
//         case 2:
//             interpol=2.0*(*(spline->m_c0));
//             break;
//         default:
//             interpol=0.0;
//             break;
//         }
//     } else if(x_position>spline->m_x[n-1]) {
//         // extrapolation to the right
//         switch(order) {
//         case 1:
//             interpol=2.0*spline->m_c[n-1]*h + spline->m_b[n-1];
//             break;
//         case 2:
//             interpol=2.0*spline->m_c[n-1];
//             break;
//         default:
//             interpol=0.0;
//             break;
//         }
//     } else {
//         // interpolation
//         switch(order) {
//         case 1:
//             interpol=(3.0*spline->m_d[idx]*h + 2.0*spline->m_c[idx])*h + spline->m_b[idx];
//             break;
//         case 2:
//             interpol=6.0*spline->m_d[idx]*h + 2.0*spline->m_c[idx];
//             break;
//         case 3:
//             interpol=6.0*spline->m_d[idx];
//             break;
//         default:
//             interpol=0.0;
//             break;
//         }
//     }
//     return interpol;
// }

// double compute_heading(double s_ref, Lanes_ISPC lanes)
// {
//     double psi_;
//     double dx = deriv(1, s_ref, lanes.spline_x);
//     double dy = deriv(1, s_ref, lanes.spline_y);
//     psi_ = atan2(dy, dx);
//     if(psi_ < 0.0) {psi_ += 2*M_PI;}
//     return psi_;
// }

/** integrates the input U to get the state X */
export void integrate_ispc(uniform double X_[], uniform const double U_[], uniform VehicleState_SOA_SIMD& s_t0_out, uniform const State_ISPC& states,
                            uniform const int traffic_num)
{
    int tu;
    int td;
    VehicleState_SIMD s_t0;
    PhysicState_SIMD u_t0;
    VehicleState_SIMD ds_t0;
    uniform int nx = nX * (N + 1);
    foreach (i = 0 ... traffic_num){
        td = nx * i;
        // Initial state:
        // uniform State_ISPC state_i = states[i];
        s_t0.x = states.x[i];
        s_t0.y = states.y[i];
        s_t0.v= states.v[i];
        s_t0.psi = states.psi[i];
        s_t0.s = 0.0;

        for (uniform int j = 0; j < N + 1; j++){
            tu = nU * (N + 1) * i + nU * j;
            td = nX * (N + 1) * i + nX * j;

            // Reference point on the center lane:
            s_t0_out.x[i*(N+1) + j] = s_t0.x;
            s_t0_out.y[i*(N+1) + j] = s_t0.y;
            s_t0_out.v[i*(N+1) + j] = s_t0.v;
            s_t0_out.psi[i*(N+1) + j] = s_t0.psi;
            s_t0_out.s[i*(N+1) + j] = s_t0.s;

            // Input control:
            u_t0.d = U_[tu + d];
            u_t0.F = U_[tu + F];
            
            // Derivatives: 
            ds_t0.x = s_t0.v * cos(s_t0.psi + cg_ratio * u_t0.d);
            ds_t0.y = s_t0.v * sin(s_t0.psi + cg_ratio * u_t0.d);
            ds_t0.v = (-1/tau) * s_t0.v + (k) * u_t0.F;
            ds_t0.psi = s_t0.v * tan(u_t0.d) * cos(cg_ratio * u_t0.d)/ length;
            ds_t0.s = s_t0.v;

            // Integration to compute the new state: 
            s_t0.x += dt * ds_t0.x;
            s_t0.y += dt * ds_t0.y;
            s_t0.v += dt * ds_t0.v;
            s_t0.psi += dt * ds_t0.psi;
            s_t0.s += dt * ds_t0.s;

            if (s_t0.v < 0.0){s_t0.v = 0.0;}

            // Save the state in the trajectory
            X_[td + x] = s_t0.x;
            X_[td + y] = s_t0.y;
            X_[td + v] = s_t0.v;
            X_[td + psi] = s_t0.psi;
            X_[td + s] = s_t0.s; 
        }
    }
}