#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);


  static constexpr double Lf = 2.67;

  // this will store the selected waypoints from MPC solve
  vector<double> pathWaypoints_x;
  vector<double> pathWaypoints_y;
};

#endif /* MPC_H */
