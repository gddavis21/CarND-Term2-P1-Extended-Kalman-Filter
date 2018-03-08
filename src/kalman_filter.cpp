#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

// KalmanFilter::KalmanFilter() {}

// KalmanFilter::~KalmanFilter() {}

// void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
//                         MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
//   x_ = x_in;
//   P_ = P_in;
//   F_ = F_in;
//   H_ = H_in;
//   R_ = R_in;
//   Q_ = Q_in;
// }

// void KalmanFilter::Predict() {
//   /**
//   TODO:
//     * predict the state
//   */
// 	x_ = F_ * x_;
// 	MatrixXd Ft = F_.transpose();
// 	P_ = F_ * P_ * Ft + Q_;
// }

// void KalmanFilter::Update(const VectorXd &z) {
//   /**
//   TODO:
//     * update the state by using Kalman Filter equations
//   */
// 	VectorXd z_pred = H_ * x_;
// 	VectorXd y = z - z_pred;
// 	MatrixXd Ht = H_.transpose();
// 	MatrixXd S = H_ * P_ * Ht + R_;
// 	MatrixXd Si = S.inverse();
// 	MatrixXd PHt = P_ * Ht;
// 	MatrixXd K = PHt * Si;

// 	//new estimate
// 	x_ = x_ + (K * y);
// 	long x_size = x_.size();
// 	MatrixXd I = MatrixXd::Identity(x_size, x_size);
// 	P_ = (I - K * H_) * P_;
// }

// void KalmanFilter::UpdateEKF(const VectorXd &z) {
//   /**
//   TODO:
//     * update the state by using Extended Kalman Filter equations
//   */
// }


/////////////////////////////////////////////

StateBelief KalmanFilter::KF(
    const StateBelief &priorBelief, 
    const VectorXd &control,
    const MatrixXd &stateTransition,
    const MatrixXd &controlTransition,
    const MatrixXd &processCovariance,
    const VectorXd &measurement,
    const MatrixXd &measTransition,
    const MatrixXd &measCovariance)
{
    VectorXd x = priorBelief.state;
    MatrixXd S = priorBelief.covariance;
    const VectorXd &u = control;
    const VectorXd &z = measurement;

    // predict belief based on prior belief & new control
    const MatrixXd &A = stateTransition;
    const MatrixXd &B = controlTransition;
    const MatrixXd &R = processCovariance;
    Matrix At = A.transpose();
    x = A*x + B*u;
    S = A*S*At + R;

    // update belief based on prediction & measurement
    const MatrixXd &C = measTransition;
    const MatrixXd &Q = measCovariance;
    MatrixXd Ct = C.transpose();
    MatrixXd I = MatrixXd::Identity(x.size(), x.size());
    MatrixXd K = S*Ct*(C*S*Ct + Q).inverse();
    x = x + K*(z - C*x);
    S = (I - K*C)*S;
    
    // return updated belief
    StateBelief newBelief;
    newBelief.state = x;
    newBelief.covariance = S;
    return newBelief;
}

StateBelief KalmanFilter::EKF_LinearPred(
    const StateBelief &priorBelief, 
    const VectorXd &control,
    const MatrixXd &stateTransition,
    const MatrixXd &controlTransition,
    const MatrixXd &processCovariance,
    const VectorXd &measurement,
    const EKF_MeasPredictor &measPredictor,
    const MatrixXd &measCovariance);
{
    VectorXd x = priorBelief.state;
    MatrixXd S = priorBelief.covariance;
    const VectorXd &u = control;
    const VectorXd &z = measurement;

    // predict belief based on prior belief & new control
    const MatrixXd &A = stateTransition;
    const MatrixXd &B = controlTransition;
    const MatrixXd &R = processCovariance;
    Matrix At = A.transpose();
    x = A*x + B*u;
    S = A*S*At + R;
    
    auto pred = measPredictor.PredictMeasurement(x);
    const VectorXd &z_pred = pred.first;
    const MatrixXd &H = pred.second;
    const MatrixXd &Q = measCovariance;
    MatrixXd Ht = H.transpose();
    MatrixXd I = MatrixXd::Identity(x.size(), x.size());
    MatrixXd K = S*Ht*(H*S*Ht + Q).inverse();
    VectorXd y = measPredictor.NormalizeResidual(z - z_pred);
    x = x + K*y;
    S = (I - K*H)*S;
    
    // return updated belief
    StateBelief newBelief;
    newBelief.state = x;
    newBelief.covariance = S;
    return newBelief;
}
