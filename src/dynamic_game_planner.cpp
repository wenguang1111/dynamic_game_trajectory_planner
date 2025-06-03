#include "dynamic_game_planner.h"
#include "recorder.h"
// #include "update_trajetcory_interface.h"
#include "integrate_ispc.h"
#include <iostream>

DynamicGamePlanner::DynamicGamePlanner() 
{
}

DynamicGamePlanner::~DynamicGamePlanner() {
    // Destructor implementation
    delete[] state_ispc; 
    std::cout << "DynamicGamePlanner destroyed." << std::endl;
}

void DynamicGamePlanner::run(TrafficParticipants& traffic_state) {
    
    traffic = traffic_state;
    // Variables initialization and setup:
    setup();
    copyDataForISPC(traffic);
    // definition of the control variable vector U and of the state vector X:
    double U[nU_];
    double X[nX_];
    double constraints[nC];

    initial_guess(X, U);
    trust_region_solver(U);
    // integrate(X, U);
    // integrate_opt(X, U);
    // integrate_SIMD(X,U);
    launch_integrate_ISPC(X, U);
    print_trajectories(X, U);
    compute_constraints(constraints, X, U);
    constraints_diagnostic(constraints, false);
    traffic = set_prediction(X, U);
    recordData(X, U); 
}

void DynamicGamePlanner::setup() {
    
    // Setup number of traffic participants:
    M = traffic.size();

    int num_block =  M%BLOCK_SIZE==0? M/BLOCK_SIZE :  (M/BLOCK_SIZE+1);
    state_ispc = (State_Block*) malloc(sizeof(State_Block)*BLOCK_SIZE*num_block);
    
    // Setup number of inequality constraints for one vehicle:
    // 2 * nU * (N + 1) inequality constraints for inputs 
    // (N + 1) * (M - 1) collision avoidance constraints
    // (N + 1) constraints to remain in the lane
    nC_i = 2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1) + (param.N + 1);

    // Setup number of inequality constraints for all the traffic participants
    nC = nC_i * M;

    // Setup number of elements in the state vector X:
    // number of state variables * number of timesteps * number of traffic participants
    nX_ = param.nX * (param.N + 1) * M;

    // Setup number of elements in the control vector U:
    // number of control variables * number of timesteps * number of traffic participants
    nU_ = param.nU * (param.N + 1) * M;

    // Setup length of the gradient vector G:
    nG = nU_;

    // resize and initialize limits for control input
    ul.resize(param.nU * (param.N + 1), 1);
    uu.resize(param.nU * (param.N + 1), 1);
    for (int j = 0; j < param.N + 1; j++){
        ul(param.nU * j + d, 0) = param.d_low;
        uu(param.nU * j + d, 0) = param.d_up;
        ul(param.nU * j + F, 0) = param.F_low;
        uu(param.nU * j + F, 0) = param.F_up;
    }

    // resize time vector
    time.resize(param.N + 1, 1);

    // resize and initialize lagrangian multiplier vector
    lagrangian_multipliers.resize(nC, 1);
    lagrangian_multipliers = Eigen::MatrixXd::Zero(nC, 1);

}

/** Sets the intial guess of the game */
void DynamicGamePlanner::initial_guess(double* X_, double* U_)
{
    for (int i = 0; i < M; i++){
        for (int j = 0; j < param.N + 1; j++){
            U_[param.nU * (param.N + 1) * i + param.nU * j + d] = 0.0;
            U_[param.nU * (param.N + 1) * i + param.nU * j + F] = 0.3;
        }
    }
    // integrate(X_, U_);
    // integrate_opt(X_,U_);
    // integrate_SIMD(X_,U_);
    launch_integrate_ISPC(X_, U_);
}

/** integrates the input U to get the state X */
void DynamicGamePlanner::integrate(double* X_, const double* U_)
{
    int tu;
    int td;
    int ind;
    double s_ref;
    double v_ref;
    double t;
    double s_t0[param.nX];
    double sr_t0[param.nX];
    double u_t0[param.nU];
    double ds_t0[param.nX];

    for (int i = 0; i < M; i++){
        ind = 0;
        td = nx * i;
        t = 0.0;

        // Initial state:
        s_t0[x] = traffic[i].x;
        s_t0[y] = traffic[i].y;
        s_t0[v] = traffic[i].v;
        s_t0[psi] = traffic[i].psi;
        s_t0[s] = 0.0;
        s_t0[l] = 0.0;

        for (int j = 0; j < param.N + 1; j++){
            tu = param.nU * (param.N + 1) * i + param.nU * j;
            td = param.nX * (param.N + 1) * i + param.nX * j;

            // Reference point on the center lane:
            s_ref = s_t0[s];
            sr_t0[x] = traffic[i].centerlane.spline_x(s_ref);
            sr_t0[y] =traffic[i].centerlane.spline_y(s_ref);
            sr_t0[psi] = traffic[i].centerlane.compute_heading(s_ref);

            // Target speed:
            v_ref = traffic[i].v_target;

            // Input control:
            u_t0[d] = U_[tu + d];
            u_t0[F] = U_[tu + F];
            
            // Derivatives: 
            ds_t0[x] = s_t0[v] * cos(s_t0[psi] + param.cg_ratio * u_t0[d]);
            ds_t0[y] = s_t0[v] * sin(s_t0[psi] + param.cg_ratio * u_t0[d]);
            ds_t0[v] = (-1/param.tau) * s_t0[v] + (param.k) * u_t0[F];
            ds_t0[psi] = s_t0[v] * tan(u_t0[d]) * cos(param.cg_ratio * u_t0[d])/ param.length;
            ds_t0[l] = param.weight_target_speed * (s_t0[v] - v_ref) * (s_t0[v] - v_ref)
                    + param.weight_center_lane * ((sr_t0[x] - s_t0[x]) * (sr_t0[x] - s_t0[x]) + (sr_t0[y] - s_t0[y]) * (sr_t0[y] - s_t0[y]))
                    + param.weight_heading * ((std::cos(sr_t0[psi]) - std::cos(s_t0[psi]))*(std::cos(sr_t0[psi]) - std::cos(s_t0[psi]))
                    +        (std::sin(sr_t0[psi]) - std::sin(s_t0[psi]))*(std::sin(sr_t0[psi]) - std::sin(s_t0[psi])))
                    + param.weight_input * u_t0[F] * u_t0[F];
            ds_t0[s] = s_t0[v];

            // Integration to compute the new state: 
            s_t0[x] += param.dt * ds_t0[x];
            s_t0[y] += param.dt * ds_t0[y];
            s_t0[v] += param.dt * ds_t0[v];
            s_t0[psi] += param.dt * ds_t0[psi];
            s_t0[s] += param.dt * ds_t0[s];
            s_t0[l] += param.dt * ds_t0[l];

            if (s_t0[v] < 0.0){s_t0[v] = 0.0;}

            // Save the state in the trajectory
            X_[td + x] = s_t0[x];
            X_[td + y] = s_t0[y];
            X_[td + v] = s_t0[v];
            X_[td + psi] = s_t0[psi];
            X_[td + s] = s_t0[s];
            X_[td + l] = s_t0[l];

            t+= param.dt;    
        }
    }
}

/** SR1 Hessian matrix update*/
void DynamicGamePlanner::hessian_SR1_update(Eigen::MatrixXd & H_, const Eigen::MatrixXd & s_, const Eigen::MatrixXd & y_, double r_)
{
    if (abs((s_.transpose() * (y_ - H_ * s_))(0,0)) > 
                r_ * (s_.transpose() * s_)(0,0) * ((y_ - H_* s_).transpose() * (y_ - H_ * s_))(0,0))
    {
        H_ = H_ + ((y_ - H_ * s_) * (y_ - H_ * s_).transpose()) / ((y_ - H_ * s_).transpose() * s_)(0,0);
    }
}

/** function to increase rho = rho * gamma at each iteration */
void DynamicGamePlanner::increasing_schedule()
{
    rho = gamma * rho;
}

/** function to save the lagrangian multipliers in the general variable */
void DynamicGamePlanner::save_lagrangian_multipliers(double* lagrangian_multipliers_)
{
    for (int i = 0; i < nC; i++){
        lagrangian_multipliers(i,0) = lagrangian_multipliers_[i];
    }
}

/* computation of lambda (without update)*/
void DynamicGamePlanner::compute_lagrangian_multipliers(double* lagrangian_multipliers_, const double* constraints_)
{
    double l;
    for (int i = 0; i < nC; i++){
        l = lagrangian_multipliers(i,0) + rho * constraints_[i];
        lagrangian_multipliers_[i] = std::max(l, 0.0);
    }
}

/** computation of the inequality constraints (target: constraints < 0) */
void DynamicGamePlanner::compute_constraints(double* constraints, const double* X_, const double* U_)
{
    double constraints_i[nC_i];
    for (int i = 0; i < M; i++){
        compute_constraints_vehicle_i(constraints_i, X_, U_, i);
        for (int j = 0; j < nC_i; j++){
            constraints[nC_i * i + j] = constraints_i[j];
        }
    }
}

/** computation of the inequality constraints C for vehicle i (target: C < 0) */
void DynamicGamePlanner::compute_constraints_vehicle_i(double* constraints_i, const double* X_, const double* U_, int i)
{
    int ind = 0;
    int indCu;
    int indCl;
    int indU;
    int indCto;
    int indClk;
    int indCuk;
    int indf;
    int n1;
    int n2;
    double dist2t[param.N + 1];
    double rad2[param.N + 1];
    double latdist2t[param.N + 1];
    double r_lane_ = param.r_lane;

    // constraints for the inputs 
    indU = param.nU * (param.N + 1) * i;
    indCu = param.nU * (param.N + 1);
    indCl = indCu + param.nU * (param.N + 1);
    for (int k = 0; k < param.N + 1; k++){
        constraints_i[param.nU * k + d] = 1e3 * (U_[indU + param.nU * k + d] - uu(param.nU * k + d,0));
        constraints_i[param.nU * k + F] = 1e3 * (U_[indU + param.nU * k + F] - uu(param.nU * k + F,0));
        constraints_i[indCu + param.nU * k + d] = 1e3 * (ul(param.nU * k + d,0) - U_[indU + param.nU * k + d]);
        constraints_i[indCu + param.nU * k + F] = 1e3 * (ul(param.nU * k + F,0) - U_[indU + param.nU * k + F]);
    }

    // collision avoidance constraints  
    for (int k = 0; k < M; k++){
        if (k != i){
            indCto = indCl + (param.N + 1) * ind;
            compute_squared_distances_vector(dist2t, X_, i, k);
            for (int j = 0; j < param.N + 1; j++){
                constraints_i[indCto + j] = (param.r_safe * param.r_safe - dist2t[j]);
            }
            ind++;
        }
    }
    indCto = indCl + (param.N + 1) * (M - 1);

    // constraints to remain in the lane
    compute_squared_lateral_distance_vector(latdist2t, X_, i);
    for (int k = 0; k < param.N + 1; k++){
        constraints_i[indCto + k] = (latdist2t[k] - r_lane_ * r_lane_);
    }
    indf = indCto + (param.N + 1);
}

/** computes a vector of the squared distance between the trajectory of vehicle i and j*/
void DynamicGamePlanner::compute_squared_distances_vector(double* squared_distances, const double* X_, int ego, int j)
{
    double x_ego;
    double y_ego;
    double x_j;
    double y_j;
    double distance;
    for (int k = 0; k < param.N + 1; k++){
        x_ego = X_[nx * ego + param.nX * k + x];
        y_ego = X_[nx * ego + param.nX * k + y];
        x_j = X_[nx * j + param.nX * k + x];
        y_j = X_[nx * j + param.nX * k + y];
        distance = (x_ego - x_j) * (x_ego - x_j) + (y_ego - y_j) * (y_ego - y_j);
        squared_distances[k] = distance;
    }
}

/** computes a vector of the squared lateral distance between the i-th trajectory and the allowed center lines at each time step*/
void DynamicGamePlanner::compute_squared_lateral_distance_vector(double* squared_distances_, const double* X_, int i)
{
    double s_;
    double x_;
    double y_;
    double x_c;
    double y_c;
    double x_r;
    double y_r;
    double x_l;
    double y_l;
    double psi_c;
    double psi_l;
    double psi_r;
    double dist_c;
    double dist_l = 1e3;
    double dist_r = 1e3;
    double dist_long_c;
    double dist_long_l;
    double dist_long_r;
    double dist2_c[param.N + 1];
    double dist2_l[param.N + 1];
    double dist2_r[param.N + 1];
    double dist2_rl_min;
    for (int j = 0; j < param.N + 1; j++){
        s_ = X_[nx * i + param.nX * j + s];
        x_ = X_[nx * i + param.nX * j + x];
        y_ = X_[nx * i + param.nX * j + y];
        dist2_c[j] = 1e3;
        dist2_l[j] = 1e3;
        dist2_r[j] = 1e3;
        if (s_ < traffic[i].centerlane.s_max){
            x_c = traffic[i].centerlane.spline_x(s_);
            y_c = traffic[i].centerlane.spline_y(s_);
            psi_c = traffic[i].centerlane.compute_heading(s_);
            dist_c = ((x_ - x_c) * (x_ - x_c) + (y_ - y_c) * (y_ - y_c));
            dist_long_c = ((x_ - x_c) * std::cos(psi_c) + (y_ - y_c) * std::sin(psi_c)) * ((x_ - x_c) * std::cos(psi_c) + (y_ - y_c) * std::sin(psi_c));
            dist2_c[j] = dist_c - dist_long_c;
        }
        if (traffic[i].leftlane.present == true && s_ < traffic[i].leftlane.s_max && traffic[i].leftlane.s_max > 10.0){
            x_l = traffic[i].leftlane.spline_x(s_);
            y_l = traffic[i].leftlane.spline_y(s_);
            psi_l = traffic[i].leftlane.compute_heading(s_);
            dist_l = ((x_ - x_l) * (x_ - x_l) + (y_ - y_l) * (y_ - y_l));
            dist_long_l = ((x_ - x_l) * std::cos(psi_l) + (y_ - y_l) * std::sin(psi_l)) * ((x_ - x_l) * std::cos(psi_l) + (y_ - y_l) * std::sin(psi_l));
            dist2_l[j] = dist_l - dist_long_l;
        }
        if (traffic[i].rightlane.present == true && s_ < traffic[i].rightlane.s_max && traffic[i].rightlane.s_max > 10.0){
            x_r = traffic[i].rightlane.spline_x(s_);
            y_r = traffic[i].rightlane.spline_y(s_);
            psi_r = traffic[i].rightlane.compute_heading(s_);
            dist_r = ((x_ - x_r) * (x_ - x_r) + (y_ - y_r) * (y_ - y_r));
            dist_long_r = ((x_ - x_r) * std::cos(psi_r) + (y_ - y_r) * std::sin(psi_r)) * ((x_ - x_r) * std::cos(psi_r) + (y_ - y_r) * std::sin(psi_r));
            dist2_r[j] = dist_r - dist_long_r;
        }
        dist2_rl_min = std::min(dist2_l[j], dist2_r[j]);
        squared_distances_[j] = std::min(dist2_rl_min, dist2_c[j]);
    }
}

/** compute the cost for vehicle i */
double DynamicGamePlanner::compute_cost_vehicle_i(const double* X_, const double* U_, int i)
{
    double final_lagrangian = X_[nx * i + param.nX * param.N + l];
    double cost = 0.5 * final_lagrangian * qf * final_lagrangian;
    return cost;
}

/** computes of the augmented lagrangian vector  L = <L_1, ..., L_M> L_i = cost_i + lagrangian_multipliers * constraints */
void DynamicGamePlanner::compute_lagrangian(double* lagrangian, const double* X_, const double* U_)
{
    double lagrangian_i;
    double cost_i;
    double constraints_i[nC_i];
    double lagrangian_multipliers_i[nC_i];
    for (int i = 0; i < M; i++){
        cost_i = compute_cost_vehicle_i( X_, U_, i);
        compute_constraints_vehicle_i(constraints_i, X_, U_, i);
        lagrangian_i = compute_lagrangian_vehicle_i( cost_i, constraints_i, i);
        lagrangian[i] = lagrangian_i;
    }
}

/** computation of the augmented lagrangian for vehicle i: lagrangian_i = cost_i + lagrangian_multipliers_i * constraints_i */
double DynamicGamePlanner::compute_lagrangian_vehicle_i(double cost_i, const double* constraints_i, int i)
{
    double lagrangian_i = cost_i;
    double constraints;
    for (int k = 0; k < nC_i; k++){
        constraints = std::max(0.0, constraints_i[k]);
        lagrangian_i += 0.5 * rho * constraints * constraints + lagrangian_multipliers(i * nC_i + k,0) * constraints_i[k];
    }
    return lagrangian_i;
}

/** computation of the gradient of lagrangian_i with respect to U_i for each i with parallelization on cpu*/
void DynamicGamePlanner::compute_gradient(double* gradient, const double* U_)
{
    // const int num_threads = std::thread::hardware_concurrency();
    const int num_threads = 1;
    std::mutex mutex;
    std::vector<std::thread> threads(num_threads);

    // Definition of the work for each thread:
    auto computeGradient = [&](int start, int end) {
        double dU[nU_];
        double dX[nX_];
        double X_[nX_];
        double lagrangian[M];
        double lagrangian_i;
        double cost_i;
        double constraints_i[nC_i];
        double lagrangian_multipliers_i[nC_i];
        int index;
        for (int i = 0; i < nU_; i++){
            dU[i] = U_[i];
        }
        // integrate(X_, U_);
        // integrate_opt(X_,U_);
        // integrate_SIMD(X_,U_);
        launch_integrate_ISPC(X_, U_);
        compute_lagrangian(lagrangian, X_, U_);
        for (int i = start; i < end; i++) {
            index = i / nu;
            dU[i] = U_[i] + param.eps;
            // integrate(dX, dU);
            // integrate_opt(dX,dU);
            // integrate_SIMD(dX,dU);
            launch_integrate_ISPC(dX, dU);
            compute_constraints_vehicle_i(constraints_i, dX, dU, index);
            cost_i = compute_cost_vehicle_i( dX, dU, index);
            lagrangian_i = compute_lagrangian_vehicle_i( cost_i, constraints_i, index);
            {
                std::lock_guard<std::mutex> lock(mutex);
                gradient[i] = (lagrangian_i - lagrangian[index]) / param.eps;
            }
            dU[i] = U_[i];
        }
    };

    // Parallelize:
    int work_per_thread = nU_ / num_threads;
    int start_index = 0;
    int end_index = 0;
    for (int i = 0; i < num_threads; ++i) {
        start_index = i * work_per_thread;
        end_index = (i == num_threads - 1) ? nU_ : start_index + work_per_thread;
        threads[i] = std::thread(computeGradient, start_index, end_index);
    }
    for (auto& thread : threads) {
        thread.join();
    }
}

/** it solves the quadratic problem (GT * s + 0.5 * sT * H * s) with solution included in the trust region ||s|| < Delta */
void DynamicGamePlanner::quadratic_problem_solver(Eigen::MatrixXd & s_, const Eigen::MatrixXd & G_, const Eigen::MatrixXd & H_, double Delta)
{
    Eigen::MatrixXd ps(nG,1);
    double tau;
    double tau_c;
    double normG;
    double GTHG;
    GTHG = (G_.transpose() * H_ * G_)(0,0);
    normG = sqrt((G_.transpose() * G_)(0,0));
    ps = - Delta * (G_ /(normG));
    if ( GTHG <= 0.0){
        tau = 1.0;
    }else{
        tau_c = (normG * normG * normG)/(Delta * GTHG);
        tau = std::min(tau_c, 1.0);
    }
    s_ = tau * ps;
}

/** prints if some constraints are violated */
void DynamicGamePlanner::constraints_diagnostic(const double* constraints, bool print = false)
{
    bool flag0 = false;
    bool flag1 = false;
    bool flag2 = false;
    bool flag3 = false;
    for (int i = 0; i < M; i++){
        flag0 = false;
        flag1 = false;
        flag2 = false;
        flag3 = false;
        for (int j = 0; j < nC_i; j++){
            if (constraints[nC_i * i +j] > 0){
                if (j < (2 * param.nU * (param.N + 1))) {
                    std::cerr<<"vehicle "<<i<<" violates input constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag0 = true;
                }
                if (j < (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1)) && j > (2 * param.nU * (param.N + 1))){
                    std::cerr<<"vehicle "<<i<<" violates collision avoidance constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag1 = true;
                }
                if (j > (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1)) && j < (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1) + (param.N + 1))){
                    std::cerr<<"vehicle "<<i<<" violates lane constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag2 = true;
                }
            }
        }
        if (print == true){
            std::cerr<<"vehicle "<<i<<"\n";
            std::cerr<<"input constraint: \n";
            for (int j = 0; j < 2 * param.nU * (param.N + 1); j++){
                std::cerr<<constraints[nC_i * i +j]<<"\t";
            }
            std::cerr<<"\ncollision avoidance constraint: \n";
            for (int j = 2 * param.nU * (param.N + 1); j < (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1)); j++){
                std::cerr<<constraints[nC_i * i +j]<<"\t";
            }
            std::cerr<<"\nlane constraint: \n";
            for (int j = (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1)); j < (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1) + (param.N + 1)); j++){
                std::cerr<<constraints[nC_i * i +j]<<"\t";
            }
            std::cerr<<"\n";
        }
    }
}

void DynamicGamePlanner::print_trajectories(const double* X, const double* U)
{
    // Define column width
    const int col_width = 12;  // Adjust this value as needed

    for (int i = 0; i < M; i++){
        std::cerr << "Vehicle: (" << traffic[i].x << ", " << traffic[i].y << ") \t" << traffic[i].v << "\n";

        // Print table header with aligned columns
        std::cerr << std::left  // Align text to the left
                  << std::setw(col_width) << "X"
                  << std::setw(col_width) << "Y"
                  << std::setw(col_width) << "V"
                  << std::setw(col_width) << "PSI"
                  << std::setw(col_width) << "S"
                  << std::setw(col_width) << "L"
                  << std::setw(col_width) << "F"
                  << std::setw(col_width) << "d"
                  << "\n";

        // Print separator line
        std::cerr << std::string(col_width * 8, '-') << "\n";

        // Print trajectory values
        for (int j = 0; j < param.N + 1; j++){
            std::cerr << std::fixed << std::left
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + x]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + y]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + v]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + psi]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + s]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + l]
                      << std::setw(col_width) << U[param.nU * (param.N + 1) * i + param.nU * j + F]
                      << std::setw(col_width) << U[param.nU * (param.N + 1) * i + param.nU * j + d]
                      << "\n";
        }
        std::cerr << "\n";
    }
}

/** computes the acceleration on the spline s(t) at time t*/
double DynamicGamePlanner::compute_acceleration(const tk::spline & spline_v, double t)
{
    return spline_v.deriv(1, t);
}

/** sets the prediction to the traffic structure*/
TrafficParticipants DynamicGamePlanner::set_prediction(const double* X_, const double* U_)
{
    TrafficParticipants traffic_ = traffic;
    for (int i = 0; i < M; i++){
        Trajectory trajectory;
        Control control;
        tk::spline spline_x;
        tk::spline spline_y;
        tk::spline spline_v;
        tk::spline spline_d;
        std::vector<double> x_;
        std::vector<double> y_;
        std::vector<double> v_;
        std::vector<double> d_;
        std::vector<double> t_;
        double time = 0.0;
        x_.push_back(traffic_[i].x);
        y_.push_back(traffic_[i].y);
        v_.push_back(traffic_[i].v);
        d_.push_back(0.5 * traffic_[i].beta);
        t_.push_back(time);
        for (int j = 0; j < param.N + 1; j++){
            time = time + param.dt;
            x_.push_back(X_[nx * i + param.nX * j + x]);
            y_.push_back(X_[nx * i + param.nX * j + y]);
            v_.push_back(X_[nx * i + param.nX * j + v]);
            d_.push_back(U_[nu * i + param.nU * j + d]);
            t_.push_back(time);
        }
        spline_x = tk::spline(t_, x_);
        spline_y = tk::spline(t_, y_);
        spline_v = tk::spline(t_, v_);
        spline_d = tk::spline(t_, d_);
        time = 0.0;

        for (int j = 0; j < param.N_interpolation; j++){
            TrajectoryPoint point;
            Input input;
            input.a = compute_acceleration(spline_v, time);
            input.delta = spline_d(time);
            point.x = spline_x(time);
            point.y = spline_y(time);
            point.psi = compute_heading(spline_x, spline_y, time);
            point.v = spline_v(time);
            point.k = compute_curvature(spline_x, spline_y, time);
            point.omega = point.k * point.v;
            point.beta = 0.5 * input.delta;
            point.t_start = time;
            point.t_end = time + param.dt_interpolation;
            trajectory.push_back(point);
            control.push_back(input);
            time += param.dt_interpolation;
        }
        traffic_[i].predicted_trajectory = trajectory;
        traffic_[i].predicted_control = control;
    }
    return traffic_;
}

/** computes the heading on the spline x(s) and y(s) at parameter s*/
double DynamicGamePlanner::compute_heading( const tk::spline & spline_x, const tk::spline & spline_y, double s)
{
    double psi;
    double dx = spline_x.deriv(1, s);
    double dy = spline_y.deriv(1, s);
    psi = atan2(dy, dx);
    if(psi < 0.0) {psi += 2*M_PI;}
    return psi;
}

/** computes the curvature on the spline x(t) and y(t) at time t*/
double DynamicGamePlanner::compute_curvature(const tk::spline & spline_x, const tk::spline & spline_y, double s)
{
    double k;
    double dx = spline_x.deriv(1, s);
    double dy = spline_y.deriv(1, s);
    double ddx = spline_x.deriv(2, s);
    double ddy = spline_y.deriv(2, s);
    k = (ddy * dx - ddx * dy) / sqrt((dx * dx + dy * dy) * (dx * dx + dy * dy) * (dx * dx + dy * dy));
    return k;
}

/** computes the norm of the gradient */
double DynamicGamePlanner::gradient_norm(const double* gradient)
{
    double norm = 0.0;
    for (int j = 0; j < nG; j++){
        norm += gradient[j] * gradient[j];
    }
    return norm;
}

/** Trust-Region solver of the dynamic game*/
void DynamicGamePlanner::trust_region_solver(double* U_)
{
    bool convergence = false;

    // Parameters:
    double eta = 1e-4;
    double r_ = 1e-8;
    double threshold_gradient_norm = M * 1e-2;
    int iter = 1;
    int iter_lim = 200;

    // Variables definition:
    double gradient[nG];
    double dU[nU_]; 
    double dU_[nU_]; 
    double dX[nX_]; 
    double dX_[nX_];
    double d_gradient[nG];
    double d_lagrangian[M];
    double lagrangian[M];
    double constraints[nC];
    double lagrangian_multipliers[nC];

    double actual_reduction[M];
    double predicted_reduction[M];
    double delta[M];
    std::vector<Eigen::MatrixXd> H_(M);
    std::vector<Eigen::MatrixXd> g_(M);
    std::vector<Eigen::MatrixXd> p_(M);
    std::vector<Eigen::MatrixXd> s_(M);
    std::vector<Eigen::MatrixXd> y_(M);

    // Variables initialization:
    // integrate(dX, U_);
    // integrate_opt(dX, U_);
    // integrate_SIMD(dX, U_);
    launch_integrate_ISPC(dX, U_);
    for (int i = 0; i < nU_; i++){
        dU[i] = U_[i];
        dU_[i] = U_[i];
    }
    for (int i = 0; i < nX_; i++){
        dX_[i] = dX[i];
    }
    for (int i = 0; i < M; i++){
        H_[i].resize(nu, nu);
        g_[i].resize(nu, 1);
        p_[i].resize(nu, 1);
        s_[i].resize(nu, 1);
        y_[i].resize(nu, 1);
        delta[i] = 1.0;
        H_[i] = Eigen::MatrixXd::Identity(nu, nu);
    }
    compute_gradient(gradient, dU_);

    // Check for convergence:
    if (gradient_norm(gradient) < threshold_gradient_norm){
        convergence = true;
    }

    // Iteration loop:
    while (convergence == false && iter < iter_lim ){

        // Compute the grandient and the lagrangian
        // integrate(dX_, dU_);
        // integrate_opt(dX_, dU_);
        // integrate_SIMD(dX_, dU_);
        launch_integrate_ISPC(dX_, dU_);
        compute_gradient(gradient, dU_);
        compute_lagrangian(lagrangian, dX_, dU_);

        // Solves the quadratic subproblem and compute the possible step dU:
        for (int i = 0; i < M; i++){
            for (int j = 0; j < param.N + 1; j++){
                g_[i](j * param.nU + d,0) = gradient[nu * i + j * param.nU + d];
                g_[i](j * param.nU + F,0) = gradient[nu * i + j * param.nU + F];
            }
            quadratic_problem_solver(s_[i], g_[i], H_[i], delta[i]);
            for (int j = 0; j < param.N + 1; j++){
                dU[nu * i + j * param.nU + d] = dU_[nu * i + j * param.nU + d] + s_[i](j * param.nU + d,0);
                dU[nu * i + j * param.nU + F] = dU_[nu * i + j * param.nU + F] + s_[i](j * param.nU + F,0);
            }
        }

        // Compute the new grandient and the new lagrangian with the possible step dU:
        // integrate(dX, dU);
        // integrate_opt(dX, dU);
        // integrate_SIMD(dX, dU);
        launch_integrate_ISPC(dX, dU);
        compute_gradient(d_gradient, dU);
        compute_lagrangian(d_lagrangian, dX, dU);

        // Check for each agent if to accept the step or not:
        for (int i = 0; i < M; i++){
            
            // Compute the actual reduction and of the predicted reduction:
            actual_reduction[i] = lagrangian[i] - d_lagrangian[i];
            predicted_reduction[i] = - (g_[i].transpose() *  s_[i] + 0.5 * s_[i].transpose() * H_[i] * s_[i])(0,0);

            // In case of very low or negative actual reduction, reject the step:
            if ( actual_reduction[i] / predicted_reduction[i] < eta){ 
                for (int j = 0; j < param.N + 1; j++){
                    dU[nu * i + j * param.nU + d] = dU_[nu * i + j * param.nU + d];
                    dU[nu * i + j * param.nU + F] = dU_[nu * i + j * param.nU + F];
                }
            }

            // In case of great reduction, and solution close to the trust region, increase the trust region:
            if ( actual_reduction[i] / predicted_reduction[i] > 0.75){ 
                if (std::sqrt((s_[i].transpose() * s_[i])(0,0)) > 0.8 * delta[i]){
                    delta[i] = 2.0 * delta[i];
                }
            }

            // In case of low actual reduction, decrease the step:
            if ( actual_reduction[i] / predicted_reduction[i] < 0.1){
                delta[i] = 0.5 * delta[i];
            }

            // Compute the difference of the gradients, then the Hessian matrix update:
            for (int j = 0; j < param.N + 1; j++){
                y_[i](j * param.nU + d,0) = d_gradient[nu * i + j * param.nU + d] - gradient[nu * i + j * param.nU + d];
                y_[i](j * param.nU + F,0) = d_gradient[nu * i + j * param.nU + F] - gradient[nu * i + j * param.nU + F];
            }
            hessian_SR1_update(H_[i], s_[i], y_[i], r_);

            // Save the solution for the next iteration:
            for (int j = 0; j < param.N + 1; j++){
                dU_[nu * i + j * param.nU + d] = dU[nu * i + j * param.nU + d];
                dU_[nu * i + j * param.nU + F] = dU[nu * i + j * param.nU + F];
            }
        }
         // Check for convergence:
        if (gradient_norm(gradient) < threshold_gradient_norm){
            convergence = true;
        }

        // Compute the new state: 
        // integrate(dX_, dU_);
        // integrate_opt(dX_, dU_);
        // integrate_SIMD(dX_, dU_);
        launch_integrate_ISPC(dX_, dU_);

        // Compute the constraints with the new solution:
        compute_constraints(constraints, dX_, dU_);

        // Compute and save in the general variable the lagrangian multipliers with the new solution:
        compute_lagrangian_multipliers(lagrangian_multipliers, constraints);
        save_lagrangian_multipliers(lagrangian_multipliers);

        // Increase the weight of the constraints in the lagrangian multipliers:
        increasing_schedule();
        iter++;
    }

    std::cerr<<"number of iterations: "<<iter<<"\n";

    //Correct the final solution:
    correctionU(dU_);

    // Save the solution:
    for(int k = 0; k < nU_; k++){
        U_[k] = dU_[k];
    }
}

void DynamicGamePlanner::correctionU(double* U_)
{
    for (int i = 0; i < M; i++){
        for (int j = 0; j < param.N + 1; j++){
            if (j == param.N){
                U_[param.nU * (param.N + 1) * i + param.nU * j + d] = U_[param.nU * (param.N + 1) * i + param.nU * (j - 1) + d];
                U_[param.nU * (param.N + 1) * i + param.nU * j + F] = U_[param.nU * (param.N + 1) * i + param.nU * (j - 1) + F];
            }
            if (U_[param.nU * (param.N + 1) * i + param.nU * j + d] > param.d_up){
                U_[param.nU * (param.N + 1) * i + param.nU * j + d] = param.d_up;
            }
            if (U_[param.nU * (param.N + 1) * i + param.nU * j + d] < param.d_low){
                U_[param.nU * (param.N + 1) * i + param.nU * j + d] = param.d_low;
            }
        }
    }
}


void DynamicGamePlanner::update_trajectory_local(int i,
                               double v_target_speed,
                               const double* U_,
                               double* sr_t0_x,
                               double* sr_t0_y,
                               double* sr_t0_psi,
                               double* ds_t0_v,
                               double* s_t0_s,
                               double* s_t0_v,
                               double* X_,
                               double* init_s_t0) 
{
    double u_t0[param.nU];
    double ds_t0[param.nX];
    // double sr_t0[param.nX];
    double s_t0[param.nX];
    // double v_ref; 
    int tu;
    int td;
    s_t0[x] = init_s_t0[x];
    s_t0[y] = init_s_t0[y];
    s_t0[v] = init_s_t0[v];
    s_t0[psi] = init_s_t0[psi];
    s_t0[s] = init_s_t0[s];
    s_t0[l] = init_s_t0[l];
    
    for (int j = 0; j < param.N + 1; j++){
        tu = param.nU * (param.N+1) * i + param.nU * j;
        td = param.nX * (param.N+1) * i + param.nX * j;

        // Reference point
        // sr_t0[x] = sr_t0_x[j];
        // sr_t0[y] = sr_t0_y[j];
        // sr_t0[psi] = sr_t0_psi[j];

        // Target speed
        // v_ref = v_target_speed;

        // Input control
        u_t0[d] = U_[tu + d];
        u_t0[F] = U_[tu + F];

        // Derivatives
        ds_t0[x] = s_t0[v] * cos(s_t0[psi] + param.cg_ratio * u_t0[d]);
        ds_t0[y] = s_t0[v] * sin(s_t0[psi] + param.cg_ratio * u_t0[d]);
        ds_t0[v] = ds_t0_v[j];
        ds_t0[psi] = s_t0[v] * tan(u_t0[d]) * cos(param.cg_ratio * u_t0[d]) / param.length;
        ds_t0[l] = param.weight_target_speed * (s_t0[v] - v_target_speed) * (s_t0[v] - v_target_speed)
                 + param.weight_center_lane * ((sr_t0_x[j] - s_t0[x]) * (sr_t0_x[j] - s_t0[x]) 
                 + (sr_t0_y[j] - s_t0[y]) * (sr_t0_y[j] - s_t0[y]))
                 + param.weight_heading * ((cos(sr_t0_psi[j]) - cos(s_t0[psi])) * (cos(sr_t0_psi[j]) - cos(s_t0[psi]))
                 + (sin(sr_t0_psi[j]) - sin(s_t0[psi])) * (sin(sr_t0_psi[j]) - sin(s_t0[psi])))
                 + param.weight_input * u_t0[F] * u_t0[F];

        // Integration
        s_t0[x] += param.dt * ds_t0[x];
        s_t0[y] += param.dt * ds_t0[y];
        s_t0[v] = s_t0_v[j];
        s_t0[psi] += param.dt * ds_t0[psi];
        s_t0[s] = s_t0_s[j];
        s_t0[l] += param.dt * ds_t0[l];

        // Save the state
        X_[td + x] = s_t0[x];
        X_[td + y] = s_t0[y];
        X_[td + v] = s_t0[v];
        X_[td + psi] = s_t0[psi];
        X_[td + s] = s_t0[s];
        X_[td + l] = s_t0[l];

        //t+= dt;
    }
}

/** integrates the input U to get the state X */
// void DynamicGamePlanner::integrate_SIMD(double* X_, const double* U_)
// {
//     int tu;
//     int td;
//     int ind;
//     double s_ref;
//     double v_ref;
//     double t;

//     std::vector<double> s_t0_s((param.N+1)*M);
//     std::vector<double> s_t0_x((param.N+1)*M);
//     std::vector<double> s_t0_y((param.N+1)*M);
//     std::vector<double> s_t0_psi((param.N+1)*M);
//     std::vector<double> s_t0_v((param.N+1)*M);

//     VehicleState_SIMD s_t0;
//     VehicleState_SIMD ds_t0;
//     PhysicState_SIMD u_t0;
//     ReferenceLaneState_SIMD sr_t0;
//     s_t0.allocate(M);
//     ds_t0.allocate(M);
//     u_t0.allocate(M);
//     sr_t0.allocate(M);

//     for (int i = 0; i < M; i++){
//         ind = 0;
//         td = nx * i;
//         // t = 0.0;

//         // Initial state:
//         s_t0.x[i] = traffic[i].x;
//         s_t0.y[i] = traffic[i].y;
//         s_t0.v[i] = traffic[i].v;
//         s_t0.psi[i] = traffic[i].psi;
//         s_t0.s[i] = 0.0;

//         for (int j = 0; j < param.N + 1; j++){
//             tu = param.nU * (param.N + 1) * i + param.nU * j; //TODO: wird automatisch von ISPC zu array umgewandelt?
//             td = param.nX * (param.N + 1) * i + param.nX * j; //TODO: wird automatisch von ISPC zu array umgewandelt?
            
//             s_t0_s[i*(param.N+1) + j] = s_t0.s[i];
//             s_t0_x[i*(param.N+1) + j] = s_t0.x[i];
//             s_t0_y[i*(param.N+1) + j] = s_t0.y[i];
//             s_t0_psi[i*(param.N+1) + j] = s_t0.psi[i];
//             s_t0_v[i*(param.N+1) + j] = s_t0.v[i];
//             // Input control:
//             u_t0.d[i] = U_[tu + d];
//             u_t0.F[i] = U_[tu + F];
            
//             // Derivatives: 
//             ds_t0.x[i] = s_t0.v[i] * cos(s_t0.psi[i] + param.cg_ratio * u_t0.d[i]);
//             ds_t0.y[i] = s_t0.v[i] * sin(s_t0.psi[i] + param.cg_ratio * u_t0.d[i]);
//             ds_t0.v[i] = (-1/param.tau) * s_t0.v[i] + (param.k)  * u_t0.F[i];
//             ds_t0.psi[i] = s_t0.v[i] * tan(u_t0.d[i]) * cos(param.cg_ratio * u_t0.d[i])/ param.length;
//             ds_t0.s[i] = s_t0.v[i];

//             // Integration to compute the new state:
//             s_t0.x[i] += param.dt * ds_t0.x[i];
//             s_t0.y[i] += param.dt * ds_t0.y[i];
//             s_t0.v[i] += param.dt * ds_t0.v[i];
//             s_t0.psi[i] += param.dt * ds_t0.psi[i];
//             s_t0.s[i] += param.dt * ds_t0.s[i];

//             if (s_t0.v[i] < 0.0){s_t0.v[i] = 0.0;}

//             // Save the state in the trajectory
//             X_[td + x] = s_t0.x[i];
//             X_[td + y] = s_t0.y[i];
//             X_[td + v] = s_t0.v[i];
//             X_[td + psi] = s_t0.psi[i];
//             X_[td + s] = s_t0.s[i];

//             // t+= param.dt; 
//         }
//     } 

//     for (int i = 0; i < M; i++){
//         s_t0.l[i] = 0.0;
//         for (int j = 0; j < param.N + 1; j++){
//             tu = param.nU * (param.N + 1) * i + param.nU * j; //TODO: wird automatisch von ISPC zu array umgewandelt?
//             td = param.nX * (param.N + 1) * i + param.nX * j;
//             int idx = i * (param.N + 1) + j;
//             s_ref = s_t0_s[idx];
//             sr_t0.x[i] = traffic[i].centerlane.spline_x(s_ref);
//             sr_t0.y[i] =traffic[i].centerlane.spline_y(s_ref);
//             sr_t0.psi[i] = traffic[i].centerlane.compute_heading(s_ref);

//             // Target speed:
//             v_ref = traffic[i].v_target;

//             // Input control:
//             u_t0.F[i] = U_[tu + F];

//             ds_t0.l[i] = param.weight_target_speed * (s_t0_v[idx] - v_ref) * (s_t0_v[idx] - v_ref)
//                     + param.weight_center_lane * ((sr_t0.x[i] - s_t0_x[idx]) * (sr_t0.x[i] - s_t0_x[idx]) + (sr_t0.y[i] - s_t0_y[idx]) * (sr_t0.y[i] - s_t0_y[idx]))
//                     + param.weight_heading * ((std::cos(sr_t0.psi[i]) - std::cos(s_t0_psi[idx]))*(std::cos(sr_t0.psi[i]) - std::cos(s_t0_psi[idx]))
//                     +        (std::sin(sr_t0.psi[i]) - std::sin(s_t0_psi[idx]))*(std::sin(sr_t0.psi[i]) - std::sin(s_t0_psi[idx])))
//                     + param.weight_input * u_t0.F[i] * u_t0.F[i];
//             s_t0.l[i] += param.dt * ds_t0.l[i];
//             X_[td + l] = s_t0.l[i];
//         }
//     }
// }

void DynamicGamePlanner::copyDataForISPC(const TrafficParticipants& traffic)
{   
    int num_block =  M%BLOCK_SIZE==0? M/BLOCK_SIZE :  (M/BLOCK_SIZE+1);
    
    for (int i = 0; i < num_block; i++){
        for(int j =0; j < BLOCK_SIZE; j++)
        {
            int index = i * BLOCK_SIZE + j;
            if (index >= M) break; // Avoid out of bounds access
            state_ispc[i].x[j] = traffic[index].x;
            state_ispc[i].y[j] = traffic[index].y;
            state_ispc[i].v[j] = traffic[index].v;
            state_ispc[i].psi[j] = traffic[index].psi;
            state_ispc[i].v_target[j] = traffic[index].v_target;
        }
    }
    // #ifdef USE_RECORDER
    //     //write me code to record the data of m_x, m_y, m_b, m_c, m_d, m_c0 and the data of get_x, get_y, get_b, get_c, get_d, get_c0
    //     for (int i = 0; i < num_block; i++) {
    //         for(int j = 0; j < BLOCK_SIZE; j++) {
    //             int index = i * BLOCK_SIZE + j;
    //             if (index >= M) break;
    //             Recorder::getInstance()->saveData<double>("x", state_ispc[i].x[j]);
    //             Recorder::getInstance()->saveData<double>("y", state_ispc[i].y[j]);
    //             Recorder::getInstance()->saveData<double>("v", state_ispc[i].v[j]);
    //             Recorder::getInstance()->saveData<double>("psi", state_ispc[i].psi[j]);
    //             Recorder::getInstance()->saveData<double>("v_target", state_ispc[i].v_target[j]);
    //             Recorder::getInstance()->saveData<double>("x_original", traffic[index].x);
    //             Recorder::getInstance()->saveData<double>("y_original", traffic[index].y);
    //             Recorder::getInstance()->saveData<double>("v_original", traffic[index].v);
    //             Recorder::getInstance()->saveData<double>("psi_original", traffic[index].psi);
    //             Recorder::getInstance()->saveData<double>("v_target_original", traffic[index].v_target);
    //             Recorder::getInstance()->saveData<double>("x_i",i);
    //             Recorder::getInstance()->saveData<double>("x_j",j);
    //         }
    //     }
    // #endif

}

void DynamicGamePlanner::convertAoSoA(double* X_, Block_X* output_X_, const double* U_, Block_U* output_U_, int num_block)
{
    for (int i = 0; i < num_block; i++)
    {
        for (int j = 0; j < NUM_State; j++)
        {
            int index = i * NUM_State + j;
            for(int k=0; k<BLOCK_SIZE;k++)
            {
                if(i* BLOCK_SIZE + k >= M) break; // Avoid out of bounds access
                int pos_U = Parameters::nU*NUM_State*(BLOCK_SIZE*i + k) + Parameters::nU*j;
                int pos_X = Parameters::nX*NUM_State*(BLOCK_SIZE*i + k) + Parameters::nX*j;
                output_X_[index].x[k] = X_[pos_X+x];
                output_X_[index].y[k] = X_[pos_X+y];
                output_X_[index].v[k] = X_[pos_X+v];
                output_X_[index].psi[k] = X_[pos_X+psi];
                output_X_[index].s[k] = X_[pos_X+s];
                output_X_[index].l[k] = X_[pos_X+l];
                output_U_[index].d[k] = U_[pos_U+d];
                output_U_[index].F[k] = U_[pos_U+F];
            }     
        }
    }
}

void DynamicGamePlanner::convertBackData_X(double* X_, Block_X* input_X_, int num_block)
{
    for (int i = 0; i < num_block; i++)
    {
        for (int j = 0; j < NUM_State; j++)
        {
            int index = i * NUM_State + j;
            for(int k=0; k<BLOCK_SIZE;k++)
            {
                if(i* BLOCK_SIZE + k >= M) break;
                int pos_X = Parameters::nX*NUM_State*(BLOCK_SIZE*i + k) + Parameters::nX*j;
                X_[pos_X+x] = input_X_[index].x[k];
                X_[pos_X+y] = input_X_[index].y[k];
                X_[pos_X+v] = input_X_[index].v[k];
                X_[pos_X+psi] = input_X_[index].psi[k];
                X_[pos_X+s] = input_X_[index].s[k];
                X_[pos_X+l] = input_X_[index].l[k];
            }     
        }
    }
}

void DynamicGamePlanner::launch_integrate_ISPC(double* X_, const double* U_)
{
    // integrate(X_, U_);
    int num_block =  M%BLOCK_SIZE==0? M/BLOCK_SIZE :  (M/BLOCK_SIZE+1);
    Block_U data_u[num_block][NUM_State];
    Block_X data_x[num_block][NUM_State];
    convertAoSoA(X_, &data_x[0][0], U_, &data_u[0][0], num_block);
    
    Block_X s_t0_data[num_block][NUM_State];
    // integrate_ispc(&data_x[0][0], &data_u[0][0], &s_t0_data[0][0], state_ispc, num_block);
    
    int tu;
    int td;
    int ind;
    double s_ref;
    double v_ref;
    
    double sr_t0[param.nX];
    
    double ds_t0[param.nX];

    Block_X s_t0;
    double u_t0[param.nX];

    for(int i = 0; i<num_block; i++){
        for(int k=0; k<BLOCK_SIZE; k++)
        {
            s_t0.x[k] = state_ispc[i].x[k];
            s_t0.y[k] = state_ispc[i].y[k];
            s_t0.v[k]= state_ispc[i].v[k];
            s_t0.psi[k] = state_ispc[i].psi[k];
            s_t0.s[k] = 0.0;
        }
        for (int j = 0; j < NUM_State; j++){
            
            int index = i * NUM_State + j;
            for ( int q = 0; q < BLOCK_SIZE; q++)
            {
                int index_obstacles = i * BLOCK_SIZE + q;
                if(index_obstacles >= M) break;

                s_t0_data[i][j].x[q] = s_t0.x[q];
                s_t0_data[i][j].y[q] = s_t0.y[q];
                s_t0_data[i][j].v[q] = s_t0.v[q];
                s_t0_data[i][j].psi[q] = s_t0.psi[q];
                s_t0_data[i][j].s[q] = s_t0.s[q];
                
                u_t0[d] = data_u[i][j].d[q];
                u_t0[F] = data_u[i][j].F[q];
                
                // Derivatives: 
                ds_t0[x] = s_t0.v[q] * cos(s_t0.psi[q] + param.cg_ratio * u_t0[d]);
                ds_t0[y] = s_t0.v[q] * sin(s_t0.psi[q] + param.cg_ratio * u_t0[d]);
                ds_t0[v] = (-1/param.tau) * s_t0.v[q] + (param.k) * u_t0[F];
                ds_t0[psi] = s_t0.v[q] * tan(u_t0[d]) * cos(param.cg_ratio * u_t0[d])/ param.length;
                ds_t0[s] = s_t0.v[q];

                s_t0.x[q] += param.dt * ds_t0[x];
                s_t0.y[q] += param.dt * ds_t0[y];
                s_t0.v[q] += param.dt * ds_t0[v];
                s_t0.psi[q] += param.dt * ds_t0[psi];
                s_t0.s[q] += param.dt * ds_t0[s];

                if (s_t0.v[q] < 0.0){s_t0.v[q] = 0.0;}

                data_x[i][j].x[q] = s_t0.x[q];
                data_x[i][j].y[q] = s_t0.y[q];
                data_x[i][j].v[q] = s_t0.v[q];
                data_x[i][j].psi[q] = s_t0.psi[q];
                data_x[i][j].s[q] = s_t0.s[q];
            }
        }
    }


    for(int i = 0; i<num_block; i++){
        double s_t0_l[BLOCK_SIZE]={0};
        for (int j = 0; j < NUM_State; j++){
            for ( int q = 0; q < BLOCK_SIZE; q++)
            {
                int index_obstacles = i * BLOCK_SIZE + q;
                if(index_obstacles >= M) break; // Avoid out of bounds access
                s_ref = s_t0_data[i][j].s[q];
                sr_t0[x] = traffic[index_obstacles].centerlane.spline_x(s_ref);
                sr_t0[y] =traffic[index_obstacles].centerlane.spline_y(s_ref);
                sr_t0[psi] = traffic[index_obstacles].centerlane.compute_heading(s_ref);

                // Target speed:
                v_ref = traffic[index_obstacles].v_target;

                // Input control:
                u_t0[d] = data_u[i][j].d[q];
                u_t0[F] = data_u[i][j].F[q];
                
                // Derivatives: 
                ds_t0[l] = param.weight_target_speed * (s_t0_data[i][j].v[q] - v_ref) * (s_t0_data[i][j].v[q]- v_ref)
                        + param.weight_center_lane * ((sr_t0[x] - s_t0_data[i][j].x[q]) * (sr_t0[x] - s_t0_data[i][j].x[q]) + (sr_t0[y] - s_t0_data[i][j].y[q]) * (sr_t0[y] - s_t0_data[i][j].y[q]))
                        + param.weight_heading * ((std::cos(sr_t0[psi]) - std::cos(s_t0_data[i][j].psi[q]))*(std::cos(sr_t0[psi]) - std::cos(s_t0_data[i][j].psi[q]))
                        +        (std::sin(sr_t0[psi]) - std::sin(s_t0_data[i][j].psi[q]))*(std::sin(sr_t0[psi]) - std::sin(s_t0_data[i][j].psi[q])))
                        + param.weight_input * u_t0[F] * u_t0[F];

                // Integration to compute the new state: 
                s_t0_l[q] += param.dt * ds_t0[l];

                // Save the state in the trajectory
                data_x[i][j].l[q] = s_t0_l[q];
            }
        }
    }
     
    convertBackData_X(X_, &data_x[0][0], num_block);
}

