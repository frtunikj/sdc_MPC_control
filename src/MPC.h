#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    vector<double> pts_x;
    vector<double> pts_y;

private:
    using Dvector = CPPAD_TESTVECTOR(double);

    void setupBoundsOnOptimizationVars(size_t size, Dvector& opt_vector_lowerbound,
            Dvector& opt_vector_upperbound) const;
    void setupBoundsOnConstraints(Dvector& constraints_lowerbound,
            Dvector& constraints_upperbound,
            size_t n_constraints, Eigen::VectorXd state) const;
};

#endif /* MPC_H */
