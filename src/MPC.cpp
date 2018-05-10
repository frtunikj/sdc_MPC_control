#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// The solver takes all the state variables and actuator
// variables in a singular vector i.e. we have to state when on
// variable starts [x0, ... xN-1, y0, ... yN-1, psi0, ... psiN-1, 
// v0, ... vN-1, cte0, ... cteN-1, epsi0,... epsiN-1, delta0, ... deltaN-2,
// a0, ... aN-2]

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
private:
    const double ref_v_ = 100.;

    // Weights used in the cost function
    const double w_cte_error_ = 2000.;
    const double w_epsi_error_ = 2000.;
    const double w_v_error_ = 2.5;
    const double w_delta_ = 100.; // 10.
    const double w_a_ = 10.;
    const double w_delta_change_ = 100;
    const double w_a_change_ = 10.;

    const double latency_ = 0.0; // seconds

public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    FG_eval(Eigen::VectorXd coeffs) {
        this->coeffs = coeffs;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars) {
        // TODO: implement MPC
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.

        fg[0] = 0;

        // The part of the cost based on the reference state.
        for (size_t t = 0; t < N; t++) {
            fg[0] += w_cte_error_ * CppAD::pow(vars[cte_start + t], 2);
            fg[0] += w_epsi_error_ * CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += w_v_error_ * CppAD::pow(vars[v_start + t] - ref_v_, 2);
        }

        // Minimize the use of actuators.
        for (size_t t = 0; t < N - 1; t++) {
            fg[0] += w_delta_ * CppAD::pow(vars[delta_start + t], 2);
            fg[0] += w_a_ * CppAD::pow(vars[a_start + t], 2);
        }

        // Minimize the value gap between sequential actuation commands.
        for (size_t t = 0; t < N - 2; t++) {
            fg[0] += w_delta_change_ * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += w_a_change_ * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }

        // Initial constraints
        // Add 1 to each of the starting indices due to cost being located atfg[0]
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];


        // The rest of the constraints
        for (size_t t = 1; t < N; t++) {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + t];
            AD<double> y1 = vars[y_start + t];
            AD<double> psi1 = vars[psi_start + t];
            AD<double> v1 = vars[v_start + t];
            AD<double> cte1 = vars[cte_start + t];
            AD<double> epsi1 = vars[epsi_start + t];

            // The state at time t.
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> y0 = vars[y_start + t - 1];
            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> v0 = vars[v_start + t - 1];
            AD<double> cte0 = vars[cte_start + t - 1];
            AD<double> epsi0 = vars[epsi_start + t - 1];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];

            AD<double> f0 = 0.0;
            for (int i = 0; i < coeffs.size(); i++) {
                f0 += coeffs[i] * CppAD::pow(x0, i);
            }

            AD<double> psides0 = 0.0;
            for (int i = 1; i < coeffs.size(); i++) {
                psides0 += i * coeffs[i] * CppAD::pow(x0, i - 1);
            }

            psides0 = CppAD::atan(psides0);

            // Setting up the rest of the model constraints
            //
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }

    }
};

//
// MPC class definition implementation.
//

MPC::MPC() {
}

MPC::~MPC() {
}

void MPC::setupBoundsOnOptimizationVars(size_t size, Dvector& opt_vector_lowerbound,
        Dvector& opt_vector_upperbound) const {
    
    // State vector part
    for (size_t i = 0; i < delta_start; i++) {
        opt_vector_lowerbound[i] = -numeric_limits<float>::max();
        opt_vector_upperbound[i] = numeric_limits<float>::max();
    }

    // Actuator variables part.
    // Steering angle [-25, 25], but expressed in radians
    double max_angle = 25.0 * M_PI / 180;
    for (size_t i = delta_start; i < a_start; i++) {
        opt_vector_lowerbound[i] = -max_angle;
        opt_vector_upperbound[i] = max_angle;
    }
    // Throttle
    for (size_t i = a_start; i < size; i++) {
        opt_vector_lowerbound[i] = -1.0;
        opt_vector_upperbound[i] = 1.0;
    }
}

void MPC::setupBoundsOnConstraints(Dvector& constraints_lowerbound,
        Dvector& constraints_upperbound,
        size_t n_constraints,
        Eigen::VectorXd state) const {
    
    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    auto x = state[0];
    auto y = state[1];
    auto psi = state[2];
    auto v = state[3];
    auto cte = state[4];
    auto epsi = state[5];

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    size_t n_vars = N * 6 + (N - 1) * 2;

    // Number of model constraints
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start] = state[0];
    vars[y_start] = state[1];
    vars[psi_start] = state[2];
    vars[v_start] = state[3];
    vars[cte_start] = state[4];
    vars[epsi_start] = state[5];

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // TODO: Set lower and upper limits for variables.

    setupBoundsOnOptimizationVars(n_vars, vars_lowerbound, vars_upperbound);

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    setupBoundsOnConstraints(constraints_lowerbound,
            constraints_upperbound,
            n_constraints, state);

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // Store x and y points
    this->pts_x = {};
    this->pts_y = {};

    for (size_t i = 0; i < N; i++) {
        this->pts_x.push_back(solution.x[x_start + i]);
        this->pts_y.push_back(solution.x[y_start + i]);
    }

    // Return the actuator values (vector = {delta, a})
    return {solution.x[delta_start], solution.x[a_start]};
}
