#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace GaussianFilters
{
    //
    // Kalman Filter state estimation algorithm
    //   as described in Probabilistic Robotics section 3.2.
    //
    StateBelief KalmanFilter(
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

        // predict new state based on prior state belief, time delta & new control
        const MatrixXd &A = stateTransition;
        const MatrixXd &B = controlTransition;
        const MatrixXd &R = processCovariance;
        MatrixXd At = A.transpose();
        x = A*x + B*u;
        S = A*S*At + R;

        // update state belief based on predicted state & new measurement
        const MatrixXd &C = measTransition;
        const MatrixXd &Q = measCovariance;
        MatrixXd Ct = C.transpose();
        MatrixXd I = MatrixXd::Identity(x.size(), x.size());
        MatrixXd K = S*Ct*(C*S*Ct + Q).inverse();
        x = x + K*(z - C*x);
        S = (I - K*C)*S;
        
        // return updated state belief
        StateBelief newBelief;
        newBelief.state = x;
        newBelief.covariance = S;
        return newBelief;
    }

    //
    // Extended Kalman Filter state estimation algorithm 
    //   as described in Probabilistic Robotics 3.3.
    //
    // This version differs from textbook EKF in that the measurement
    // update is nonlinear, but the state prediction is linear. Textbook
    // EKF is nonlinear in both.
    //
    StateBelief ExtendedKalmanFilter_LinearPred(
        const StateBelief &priorBelief, 
        const VectorXd &control,
        const MatrixXd &stateTransition,
        const MatrixXd &controlTransition,
        const MatrixXd &processCovariance,
        const VectorXd &measurement,
        const EKF_MeasPredictor &measPredictor,
        const MatrixXd &measCovariance)
    {
        VectorXd x = priorBelief.state;
        MatrixXd S = priorBelief.covariance;
        const VectorXd &u = control;
        const VectorXd &z = measurement;

        // predict new state based on prior state belief & new control
        const MatrixXd &A = stateTransition;
        const MatrixXd &B = controlTransition;
        const MatrixXd &R = processCovariance;
        MatrixXd At = A.transpose();
        x = A*x + B*u;
        S = A*S*At + R;
        
        // update state belief based on predicted state & new measurement
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
}
