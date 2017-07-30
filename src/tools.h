#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * Calculates the RMSE - Root Mean Squared Error.
   */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
  
  /**
   * Calculated the NIS - Normalized Innovation Squared.
   */
  double CalculateNIS(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S);
};

#endif /* TOOLS_H_ */
