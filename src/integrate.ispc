#include "ispc_parameter.h"

# define M_PI		3.14159265358979323846	/* pi */

struct Spline_ISPC
{
    const double* m_x;
    const double* m_y;
    const double* m_b;
    const double* m_c;
    const double* m_d;
    const double* m_c0;
};

struct Lanes_ISPC{
    Spline_ISPC* spline_x;
    Spline_ISPC* spline_y;
};

struct State_ISPC{
    double x;
    double y;
    double v;
    double psi;
    double v_target;
};

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

int upper_bound(uniform const double* arr, int size, double value) {
    int low = 0, high = size;
    while (low < high) {
        int mid = (low + high) >> 1;
        if (arr[mid] <= value)
            low = mid + 1;
        else
            high = mid;
    }
    return low;
}

size_t find_closest(double target_x, uniform Spline_ISPC* varying spline, size_t size)
{
    int it = upper_bound(spline->m_x, size, target_x);       // *it > x
    size_t idx = max((int)(it-spline->m_x[0]-1), 0);   // m_x[idx] <= x
    return idx;
}

double calculate_position(double s_ref, uniform Spline_ISPC* spline)
{
    size_t n=sizeof(spline->m_x)/sizeof(spline->m_x[0]);
    size_t idx=find_closest(s_ref,spline, n);

    double h=x-spline->m_x[idx];
    double interpol;
    if(x<spline->m_x[0]) {
        // extrapolation to the left
        interpol=(*(spline->m_c0)*h + spline->m_b[0])*h + spline->m_y[0];
    } else if(x>spline->m_x[n-1]) {
        // extrapolation to the right
        interpol=(spline->m_c[n-1]*h + spline->m_b[n-1])*h + spline->m_y[n-1];
    } else {
        // interpolation
        interpol=((spline->m_d[idx]*h + spline->m_c[idx])*h + spline->m_b[idx])*h + spline->m_y[idx];
    }
    return interpol;
}

double deriv(int order, double x_position, uniform Spline_ISPC* spline)
{
    //assert(order>0);
    size_t n = sizeof(spline->m_x)/sizeof(spline->m_x[0]);
    size_t idx = find_closest(x_position, spline, n);

    double h=x_position-spline->m_x[idx];
    double interpol;
    if(x_position<spline->m_x[0]) {
        // extrapolation to the left
        switch(order) {
        case 1:
            interpol=2.0*(*(spline->m_c0))*h + spline->m_b[0];
            break;
        case 2:
            interpol=2.0*(*(spline->m_c0));
            break;
        default:
            interpol=0.0;
            break;
        }
    } else if(x_position>spline->m_x[n-1]) {
        // extrapolation to the right
        switch(order) {
        case 1:
            interpol=2.0*spline->m_c[n-1]*h + spline->m_b[n-1];
            break;
        case 2:
            interpol=2.0*spline->m_c[n-1];
            break;
        default:
            interpol=0.0;
            break;
        }
    } else {
        // interpolation
        switch(order) {
        case 1:
            interpol=(3.0*spline->m_d[idx]*h + 2.0*spline->m_c[idx])*h + spline->m_b[idx];
            break;
        case 2:
            interpol=6.0*spline->m_d[idx]*h + 2.0*spline->m_c[idx];
            break;
        case 3:
            interpol=6.0*spline->m_d[idx];
            break;
        default:
            interpol=0.0;
            break;
        }
    }
    return interpol;
}

double compute_heading(double s_ref, Lanes_ISPC lanes)
{
    double psi_;
    double dx = deriv(1, s_ref, lanes.spline_x);
    double dy = deriv(1, s_ref, lanes.spline_y);
    psi_ = atan2(dy, dx);
    if(psi_ < 0.0) {psi_ += 2*M_PI;}
    return psi_;
}

/** integrates the input U to get the state X */
export void integrate_ispc(uniform double X_[], uniform const double U_[], uniform Lanes_ISPC lanes[], uniform State_ISPC states[],
                            uniform int traffic_num)
{
    int tu;
    int td;
    double s_ref;
    double v_ref;
    double s_t0[nX];
    double sr_t0[nX];
    double u_t0[nU];
    double ds_t0[nX];

    foreach (i = 0 ... traffic_num){
        // Initial state:
        State_ISPC state_i = states[i];
        Lanes_ISPC lane_i = lanes[i];
        s_t0[x] = state_i.x;
        s_t0[y] = state_i.y;
        s_t0[v] = state_i.v;
        s_t0[psi] = state_i.psi;
        s_t0[s] = 0.0;
        s_t0[l] = 0.0;

        for (uniform int j = 0; j < N + 1; j++){
            tu = nU * (N + 1) * i + nU * j;
            td = nX * (N + 1) * i + nX * j;

            // Reference point on the center lane:
            s_ref = s_t0[s];
            sr_t0[x] = calculate_position(s_ref, lane_i.spline_x);
            sr_t0[y] = calculate_position(s_ref, lane_i.spline_y);
            sr_t0[psi] = compute_heading(s_ref, lane_i);

            // Target speed:
            v_ref = state_i.v_target;

            // Input control:
            u_t0[d] = U_[tu + d];
            u_t0[F] = U_[tu + F];
            
            // Derivatives: 
            ds_t0[x] = s_t0[v] * cos(s_t0[psi] + cg_ratio * u_t0[d]);
            ds_t0[y] = s_t0[v] * sin(s_t0[psi] + cg_ratio * u_t0[d]);
            ds_t0[v] = (-1/tau) * s_t0[v] + (k) * u_t0[F];
            ds_t0[psi] = s_t0[v] * tan(u_t0[d]) * cos(cg_ratio * u_t0[d])/ length;
            ds_t0[l] = weight_target_speed * (s_t0[v] - v_ref) * (s_t0[v] - v_ref)
                    + weight_center_lane * ((sr_t0[x] - s_t0[x]) * (sr_t0[x] - s_t0[x]) + (sr_t0[y] - s_t0[y]) * (sr_t0[y] - s_t0[y]))
                    + weight_heading * ((cos(sr_t0[psi]) - cos(s_t0[psi]))*(cos(sr_t0[psi]) - cos(s_t0[psi]))
                    +        (sin(sr_t0[psi]) - sin(s_t0[psi]))*(sin(sr_t0[psi]) - sin(s_t0[psi])))
                    + weight_input * u_t0[F] * u_t0[F];
            ds_t0[s] = s_t0[v];

            // Integration to compute the new state: 
            s_t0[x] += dt * ds_t0[x];
            s_t0[y] += dt * ds_t0[y];
            s_t0[v] += dt * ds_t0[v];
            s_t0[psi] += dt * ds_t0[psi];
            s_t0[s] += dt * ds_t0[s];
            s_t0[l] += dt * ds_t0[l];

            if (s_t0[v] < 0.0){s_t0[v] = 0.0;}

            // Save the state in the trajectory
            X_[td + x] = s_t0[x];
            X_[td + y] = s_t0[y];
            X_[td + v] = s_t0[v];
            X_[td + psi] = s_t0[psi];
            X_[td + s] = s_t0[s];
            X_[td + l] = s_t0[l];   
        }
    } 
}