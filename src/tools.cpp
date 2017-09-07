#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  
  // Creating an rmse vector with length of 4 and initializing values to 0
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  
  // Checking if Esitmation Vector length is not equal to ground truth vector length
  // or if Estimation Vector length is equal to 0
  if (estimations.size() != ground_truth.size() || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
	return rmse;
  }
  
  // Calculating difference squared of each value
  for (unsigned int i = 0; i < estimations.size(); ++i){
    
	VectorXd residual = estimations[i] - ground_truth[i];
	
	// Element-wise multiplication to obtain squares of each data pointer
	residual = residual.array() * residual.array();
	rmse += residual;
	
  }
  
  // Calculating reduced mean of the same
  rmse = rmse/estimations.size();
  
  // Calculating the squared root
  rmse = rmse.array().sqrt();
  
  // Returning the result
  return rmse;
}