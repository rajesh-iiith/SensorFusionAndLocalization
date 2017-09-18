#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N  = 10;
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
double refCte = 0;
double refEpsi = 0;
double refV = 50;
size_t xStart = 0;
size_t yStart = xStart + N;
size_t psiStart = yStart + N;
size_t vStart = psiStart + N;
size_t cteStart = vStart + N;
size_t epsiStart = cteStart + N;
size_t deltaStart = epsiStart + N;
size_t aStart = deltaStart + N - 1;

class FG_eval {
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    for (size_t i = 0; i < N; i++) {
      fg[0] += CppAD::pow(vars[cteStart + i] - refCte, 2);
      fg[0] += CppAD::pow(vars[epsiStart + i] - refEpsi, 2);
      fg[0] += CppAD::pow(vars[vStart + i] - refV, 2);
    }

    for (size_t i = 0; i < N - 1; i++) {
      fg[0] += CppAD::pow(vars[deltaStart + i], 2);
      fg[0] += CppAD::pow(vars[aStart + i], 2);
    }

    for (size_t i = 0; i < N - 2; i++) {
      fg[0] += 200 * CppAD::pow(vars[deltaStart + i + 1] - vars[deltaStart + i], 2);
      fg[0] += CppAD::pow(vars[aStart + i + 1] - vars[aStart + i], 2);
    }

    fg[1 + xStart] = vars[xStart];
    fg[1 + yStart] = vars[yStart];
    fg[1 + psiStart] = vars[psiStart];
    fg[1 + vStart] = vars[vStart];
    fg[1 + cteStart] = vars[cteStart];
    fg[1 + epsiStart] = vars[epsiStart];

    for (size_t i = 0; i < N - 1; i++) {
      //state at time t+1 .
      AD<double> x1 = vars[xStart + i + 1];
      AD<double> y1 = vars[yStart + i + 1];
      AD<double> psi1 = vars[psiStart + i + 1];
      AD<double> v1 = vars[vStart + i + 1];
      AD<double> cte1 = vars[cteStart + i + 1];
      AD<double> epsi1 = vars[epsiStart + i + 1];

      //state at time t.
      AD<double> x0 = vars[xStart + i];
      AD<double> y0 = vars[yStart + i];
      AD<double> psi0 = vars[psiStart + i];
      AD<double> v0 = vars[vStart + i];
      AD<double> cte0 = vars[cteStart + i];
      AD<double> epsi0 = vars[epsiStart + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[deltaStart + i];
      AD<double> a0 = vars[aStart + i];

      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);

      fg[2 + xStart + i]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + yStart + i]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psiStart + i]  = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + vStart + i]    = v1 - (v0 + a0 * dt);
      fg[2 + cteStart + i]  = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[2 + epsiStart + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
    //End of TODO-1
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  double x    = x0[0];
  double y    = x0[1];
  double psi  = x0[2];
  double v    = x0[3];
  double cte  = x0[4];
  double epsi = x0[5];
  size_t n_vars =  N * 8 - 2;
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  for (size_t i = 0; i < deltaStart; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (size_t i = deltaStart; i < aStart; i++) {
    vars_lowerbound[i] = -25 * M_PI / 180;
    vars_upperbound[i] = 25 * M_PI / 180;
  }

  for (size_t i = aStart; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[xStart]    = x;
  constraints_lowerbound[yStart]    = y;
  constraints_lowerbound[psiStart]  = psi;
  constraints_lowerbound[vStart]    = v;
  constraints_lowerbound[cteStart]  = cte;
  constraints_lowerbound[epsiStart] = epsi;

  constraints_upperbound[xStart]    = x;
  constraints_upperbound[yStart]    = y;
  constraints_upperbound[psiStart]  = psi;
  constraints_upperbound[vStart]    = v;
  constraints_upperbound[cteStart]  = cte;
  constraints_upperbound[epsiStart] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more prsize_t information
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

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  vector<double> out_result;
  out_result.push_back(solution.x[deltaStart]);
  out_result.push_back(solution.x[aStart]);

  for (size_t i = 1; i < N; i++) {
    out_result.push_back(solution.x[xStart + i]);
    out_result.push_back(solution.x[yStart + i]);
  }
  return out_result; 
}