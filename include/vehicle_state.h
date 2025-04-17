#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include <vector>
#include "spline.h"  // Include the third-party spline library

struct TrajectoryPoint {
    double x;       /** x coordinate */
    double y;       /** y coordinate */
    double psi;     /** heading */
    double v;       /** speed */
    double omega;   /** yaw rate */
    double beta;    /** slip angle */
    double k;       /** curvature */
    double s;       /** progression */   
    double t_start; /** starting time of the point*/
    double t_end;   /** ending time of the point*/
};

struct TrajectoryPointSoA {
    double* x;       /** x coordinate */
    double* y;       /** y coordinate */
    double* psi;     /** heading */
    double* v;       /** speed */
    double* omega;   /** yaw rate */
    double* beta;    /** slip angle */
    double* k;       /** curvature */
    double* s;       /** progression */   
    double* t_start; /** starting time of the point*/
    double* t_end;   /** ending time of the point*/
};

// typedef TrajectoryPointSoA Trajectory;
typedef std::vector<TrajectoryPoint> Trajectory;

struct Input {
    double a;       /** acceleration */
    double delta;   /** steering angle */     
};
struct InputSoA {
    double* a;       /** acceleration */
    double* delta;   /** steering angle */     
};
// typedef InputSoA Control;
typedef std::vector<Input> Control;

/** Lane structure to store lane properties */
struct Lane {
    bool present;                   /** Indicates if the lane is present */
    double s_max;                   /** Maximum longitudinal position in the lane */
    tk::spline spline_x;            /** Spline for x-coordinates */
    tk::spline spline_y;            /** Spline for y-coordinates */

    Lane() : present(false), s_max(0.0) {}  // Default constructor

    void initialize_spline(const std::vector<double>& x, 
                          const std::vector<double>& y, 
                          const std::vector<double>& s);
    double compute_heading(double s);
    double compute_curvature(double s);
};

/** Vehicle state representation */
struct VehicleState {
    double x;                               /** X position */
    double y;                               /**  Y position */
    double psi;                             /** heading of the vehicle */
    double beta;                            /** slip angle*/
    double v;                               /** speed of the vehicle */
    double a;                               /** acceleration */
    double L;                               /** length of the i-th vehicle */
    double W;                               /** width of the i-th vehicle */
    double v_target;                        /** target speed of the i-th vehicle */

    Lane centerlane;                        /** Center lane */
    Lane leftlane;                          /** Left lane */
    Lane rightlane;                         /** Right lane */

    Trajectory predicted_trajectory;        /** predicted trajectory*/
    Control predicted_control;              /** predicted control*/

    VehicleState(double x_, double y_, double v_, double psi_, double beta_, double a_, double v_target_)
        : x(x_), y(y_), v(v_), psi(psi_), beta(beta_), a(a_), v_target(v_target_) {}
};

// SoA version of VehicleState
struct VehicleStateSoA {
    double* x;
    double* y;
    double* psi;
    double* beta;
    double* v;
    double* a;
    double* L;
    double* W; 
    double* v_target;
    Lane* centerlane;
    Lane* leftlane;
    Lane* rightlane; 
    Trajectory* predicted_trajectory;
    Control* predicted_control;

    int size; // number of vehicles

    // VehicleStateSoA(int n) : size(n) {
    //     x = new float[n]();
    //     y = new float[n]();
    //     v = new float[n]();
    //     psi = new float[n]();
    //     s = new float[n]();
    //     l = new float[n]();
    //     v_target = new float[n]();
    //     centerlane = new Spline[n]();
    // }

    // ~VehicleStateSoA() {
    //     delete[] x;
    //     delete[] y;
    //     delete[] v;
    //     delete[] psi;
    //     delete[] s;
    //     delete[] l;
    //     delete[] v_target;
    //     delete[] centerlane;
    // }

    // // Optional: copy from AoS
    // void fromAoS(const std::vector<VehicleState>& aos) {
    //     for (int i = 0; i < size; ++i) {
    //         x[i] = aos[i].x;
    //         y[i] = aos[i].y;
    //         v[i] = aos[i].v;
    //         psi[i] = aos[i].psi;
    //         s[i] = aos[i].s;
    //         l[i] = aos[i].l;
    //         v_target[i] = aos[i].v_target;
    //         centerlane[i] = aos[i].centerlane;
    //     }
    // }

    // // Optional: copy back to AoS
    // void toAoS(std::vector<VehicleState>& aos) const {
    //     for (int i = 0; i < size; ++i) {
    //         aos[i].x = x[i];
    //         aos[i].y = y[i];
    //         aos[i].v = v[i];
    //         aos[i].psi = psi[i];
    //         aos[i].s = s[i];
    //         aos[i].l = l[i];
    //         aos[i].v_target = v_target[i];
    //         aos[i].centerlane = centerlane[i];
    //     }
    // }
};


// using TrafficParticipants = VehicleStateSoA;  // Alias for a list of vehicles
using TrafficParticipants = std::vector<VehicleState>; 

#endif  // VEHICLE_STATE_H