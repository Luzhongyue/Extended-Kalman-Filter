#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse<<0,0,0,0;
  //the estimations size should not be zero
  if (estimations.size()==0) {
    cout<<"the stimations size should not be zero"<<endl;
    return rmse;
  
  }
  //the estimation vector size should equal ground truth vector size
  if (estimations.size()!=ground_truth.size()) {
    cout<<"the estimation vector size should equal ground truth vector size"<<endl;
    return rmse;
  }
  
  //accumulate squared residuals
  for (unsigned int i=0;i < estimations.size();++i) {
    VectorXd residuals=estimations[i]-ground_truth[i];
    residuals=residuals.array()*residuals.array();
    rmse+=residuals;
  }
  //calculate the mean
  rmse=rmse/estimations.size();
  //calculate the squared root
  rmse=rmse.array().sqrt();
  return rmse;
    
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px=x_state(0);
  float py=x_state(1);
  float vx=x_state(2);
  float vy=x_state(3);
  
  float eps=0.0001;
  const float d1=std::max(eps,px*px+py*py);
  float d2=sqrt(d1);
  float d3=d1*d2;

  //compute the Jacobian matrix
  Hj<<px/d2,py/d2,0,0,
     -py/d1,px/d1,0,0,
     py*(vx*py-vy*px)/d3,px*(vy*px-vx*py)/d3,px/d2,py/d2;
  return Hj;
}
