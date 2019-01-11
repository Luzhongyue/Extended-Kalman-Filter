#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
const float PI2=2*M_PI;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_=F_*x_;
  MatrixXd Ft=F_.transpose();
  P_=F_*P_*Ft+Q_;
}
void KalmanFilter::UpdateCommon(const VectorXd &y) {
  MatrixXd Ht=H_.transpose();
  MatrixXd S=H_*P_*Ht+R_;
  MatrixXd Si=S.inverse();
  MatrixXd K=P_*Ht*Si;
  //new estimate
  x_=x_+K*y;
  P_-=K*H_*P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  VectorXd y=z-H_*x_;
  //new estimate
  UpdateCommon(y);
}

VectorXd RadarCartesian2Polar(const VectorXd &x_state){
  float px,py,vx,vy,rho,phi,rho_dot;
  px=x_state(0);
  py=x_state(1);
  vx=x_state(2);
  vy=x_state(3);
  
  rho=sqrt(px*px+py*py);
  phi=atan2(py,px);
  //avoid divide by zero
  if (rho<0.00001) {
    rho=0.00001;
  }
  rho_dot=(px*vx+py*vy)/rho;
  VectorXd h(3);
  h<<rho,phi,rho_dot;
  return h;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd h=RadarCartesian2Polar(x_);
  VectorXd y=z-h;
  //normalize the angle between -pi and pi
  while (y(1)>M_PI) {
    y(1)-=PI2;
  }
  while (y(1)<-M_PI) {
    y(1)+=PI2;
  }
  //new estimate
  UpdateCommon(y);
  
}
