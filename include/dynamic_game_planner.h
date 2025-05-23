#ifndef DYNAMIC_GAME_PLANNER_H
#define DYNAMIC_GAME_PLANNER_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <iomanip>
#include <mutex>
#include "vehicle_state.h"
#include "utils.h"  // Utility functions
#include "parameters.h"  // Parameters for the planner
#include "integrate_ispc.h"
#include "recorder.h"

using namespace ispc;
class DynamicGamePlanner {
public:
    static constexpr int nx = Parameters::nX * (Parameters::N + 1);                          /** size of the state trajectory X_i for each vehicle */
    static const int nu = Parameters::nU * (Parameters::N + 1);                                 /** size of the input trajectory U_i for each vehicle */
    int M;                                                              /** number of agents */ 
    int nC;                                                             /** total number of inequality constraints */
    int nC_i;                                                           /** inequality constraints for one vehicle */
    int nG;                                                             /** number of elements in the gradient G */
    int nX_;                                                            /** number of elements in the state vector X */
    int nU_;                                                            /** number of elements in the input vector U */
    int M_old;                                                          /** number of traffic participants in the previous iteration*/
    
    // Parameters:
    double rho = 1e-3;                                                  /** penalty weight */ 
    double qf = 1e-2;                                                   /** penalty for the final error in the lagrangian */
    double gamma = 1.3;                                                 /** increasing factor of the penalty weight */

    std::vector<double> U_old;                                          /** solution in the previous iteration*/
    Eigen::MatrixXd ul;                                                 /** controls lower bound*/
    Eigen::MatrixXd uu;                                                 /** controls upper bound*/
    Eigen::MatrixXd time;                                               /** time vector */
    Eigen::MatrixXd lagrangian_multipliers;                             /** lagrangian multipliers*/
    Parameters param;

    TrafficParticipants traffic;
    Lanes_ISPC lanes_ispc;
    State_ISPC state_ispc;

    DynamicGamePlanner();  // Constructor
    ~DynamicGamePlanner(); // Destructor

    void run( TrafficParticipants& traffic_state );                                  /** Main method to execute the planner */
    void setup();                                                                  /** Setup function */
    void initial_guess(double* X, double* U);                                      /** Set the initial guess */
    void trust_region_solver(double* U_);                                          /** solver of the dynamic game based on trust region */
    void integrate(double* X, const double* U);                                          /** Integration function */
    void hessian_SR1_update( Eigen::MatrixXd & H_, const Eigen::MatrixXd & s_,            
                     const Eigen::MatrixXd & y_, const double r_ );                /** SR1 Hessian matrix update*/
    void increasing_schedule();                                                    /** function to increase rho = rho * gamma */
    void save_lagrangian_multipliers(double* lagrangian_multipliers_);             /** function to save the lagrangian multipliers */
    void compute_lagrangian_multipliers(double* lagrangian_multipliers_, 
                                        const double* constraints_);               /** computation of the lagrangian multipliers */
    
    void compute_constraints(double* constraints, const double* X_, 
                            const double* U_);                                     /** computation of the inequality constraints */
    void compute_constraints_vehicle_i(double* C_i, 
                            const double* X_, const double* U_, int i);            /** computation of the inequality constraints 
                                                                                        for vehicle i */
    void compute_squared_distances_vector(double* squared_distances_, const double* X_, 
                            int ego, int j);                                       /** computes a vector of the squared distance 
                                                                                        between the trajectory of vehicle i and j*/
    void compute_safety_radius(double* r2_, const double* X_, 
                            int ego, int j);                                       /** computes the safety radius between the ego vehicle 
                                                                                        and vehicle j as vector in time */
    void compute_squared_lateral_distance_vector(double* squared_distances_, 
                            const double* X_, int i);                               /** computes a vector of the squared lateral distance 
                                                                                        between the i-th trajectory and the allowed center 
                                                                                        lines at each time step*/
    double compute_cost_vehicle_i(const double* X_, const double* U_, int i);         /** compute the cost for vehicle i */
    void compute_lagrangian(double* lagrangian, 
                            const double* X_, const double* U_);                    /** computes of the augmented lagrangian vector 
                                                                                    L = <L_1, ..., L_M> 
                                                                                    L_i = cost_i + lagrangian_multipliers * constraints */
    double compute_lagrangian_vehicle_i(double J_i, const double* C_i, int i);      /** computation of the augmented lagrangian for vehicle i: 
                                                                                lagrangian_i = cost_i + lagrangian_multipliers_i * constraints_i */
    void compute_gradient(double* gradient, const double* U_);                      /** computes the gradient of lagrangian_i with respect to 
                                                                                    U_i for each i */
    void quadratic_problem_solver(Eigen::MatrixXd & s_, 
                                const Eigen::MatrixXd & G_, 
                                const Eigen::MatrixXd & H_, double Delta);          /** it solves the quadratic problem 
                                                                                        (GT * s + 0.5 * sT * H * s) with solution included in the 
                                                                                        trust region ||s|| < Delta */
    void constraints_diagnostic(const double* constraints, bool print);             /** shows violated constraints */
    void print_trajectories(const double* X, const double* U);                      /** prints trajectories */
    double compute_acceleration(const tk::spline & spline_v, double t);              /** computes the acceleration on the spline s(t) at time t*/
    TrafficParticipants set_prediction(const double* X_, const double* U_);         /** sets the prediction to the traffic structure */
    double compute_heading(const tk::spline & spline_x, 
                           const tk::spline & spline_y, double s);                  /** computes the heading on the spline x(s) and y(s) at parameter s */
    double compute_curvature(const tk::spline & spline_x, 
                             const tk::spline & spline_y, double s);                 /** computes the curvature on the spline x(t) and y(t) at time t*/
    double gradient_norm(const double* gradient);                                               /** computes the norm of the gradient */
    void correctionU(double* U_);                                                    /** corrects U if outside the boundaries */
    void integrate_opt(double* X, const double* U); 
    void copyDataForISPC(const TrafficParticipants& traffic);
    void launch_integrate_ISPC(double* X_, const double* U_);
    void integrate_SIMD(double* X_, const double* U_);
    void update_trajectory_local(int i,
        double v_target_speed,
        const double* U_,
        double* sr_t0_x,
        double* sr_t0_y,
        double* sr_t0_psi,
        double* ds_t0_v,
        double* s_t0_s,
        double* s_t0_v,
        double* X_,
        double* init_s_t0); 
    void recordData(const double* X_, const double* U_)
        {
            #ifdef USE_RECORDER
            for (int i = 0; i < M; i++) {
                for (int j = 0; j < param.N + 1; j++) {
                Recorder::getInstance()->saveData<double>("i", i);
                Recorder::getInstance()->saveData<double>("j", j);    
                Recorder::getInstance()->saveData<double>("X_x", X_[param.nX * (param.N + 1) * i + param.nX * j + x]);
                Recorder::getInstance()->saveData<double>("X_y", X_[param.nX * (param.N + 1) * i + param.nX * j + y]);
                Recorder::getInstance()->saveData<double>("X_v", X_[param.nX * (param.N + 1) * i + param.nX * j + v]);
                Recorder::getInstance()->saveData<double>("X_psi", X_[param.nX * (param.N + 1) * i + param.nX * j + psi]);
                Recorder::getInstance()->saveData<double>("X_s", X_[param.nX * (param.N + 1) * i + param.nX * j + s]);
                Recorder::getInstance()->saveData<double>("X_l", X_[param.nX * (param.N + 1) * i + param.nX * j + l]);
                Recorder::getInstance()->saveData<double>("U_F", U_[param.nU * (param.N + 1) * i + param.nU * j + F]);
                Recorder::getInstance()->saveData<double>("U_d", U_[param.nU * (param.N + 1) * i + param.nU * j + d]);
                }
            }
            #endif
        }
};

// struct PhysicState_SIMD
// {
//     double* d = nullptr;
//     double* F = nullptr;
//     size_t size = 0;

//     PhysicState_SIMD() = default;

//     PhysicState_SIMD(const PhysicState_SIMD&) = delete;
//     PhysicState_SIMD& operator=(const PhysicState_SIMD&) = delete;

//     PhysicState_SIMD(PhysicState_SIMD&& other) noexcept
//     {
//         *this = std::move(other);
//     }

//     PhysicState_SIMD& operator=(PhysicState_SIMD&& other) noexcept
//     {
//         if (this != &other)
//         {
//             d = other.d; 
//             F = other.F;
//             size = other.size;

//             other.d = other.F = nullptr;
//             other.size = 0;
//         }
//         return *this;
//     }

//     ~PhysicState_SIMD()
//     {
//         freeMemory();
//     }

//     void allocate(size_t n)
//     {
//         freeMemory();
//         size = n;

//         d = allocateArray(n);
//         F = allocateArray(n);
//     }

// private:
//     static double* allocateArray(size_t n)
//     {
//         void* ptr = std::aligned_alloc(ALIGNMENT, n * sizeof(double));
//         if (!ptr)
//         {
//             throw std::runtime_error("aligned_alloc failed!");
//         }
//         std::memset(ptr, 0, n * sizeof(double));
//         return static_cast<double*>(ptr);
//     }

//     void freeMemory()
//     {
//         std::free(d);
//         std::free(F);

//         d = F = nullptr;
//         size = 0;
//     }
// };
#endif // DYNAMIC_GAME_PLANNER_H
