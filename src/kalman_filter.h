#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include <utility>
#include "Eigen/Dense"

namespace GaussianFilters
{
    struct StateBelief
    {
        Eigen::VectorXd state;
        Eigen::MatrixXd covariance;
    };

    // Kalman Filter state estimation
    StateBelief KalmanFilter(
        const StateBelief &priorBelief, 
        const Eigen::VectorXd &control,
        const Eigen::MatrixXd &stateTransition,
        const Eigen::MatrixXd &controlTransition,
        const Eigen::MatrixXd &processCovariance,
        const Eigen::VectorXd &measurement,
        const Eigen::MatrixXd &measTransition,
        const Eigen::MatrixXd &measCovariance);

    class EKF_MeasPredictor;  // client-supplied nonlinear transformation

    // Extended Kalman Filter state estimation 
    // This version differs from typical EKF by combining a linear state
    // prediction with a nonlinear measurement update.
    StateBelief ExtendedKalmanFilter_LinearPred(
        const StateBelief &priorBelief,
        const Eigen::VectorXd &control,
        const Eigen::MatrixXd &stateTransition,
        const Eigen::MatrixXd &controlTransition,
        const Eigen::MatrixXd &processCovariance,
        const Eigen::VectorXd &measurement,
        const EKF_MeasPredictor &measPredictor,
        const Eigen::MatrixXd &measCovariance);

    // Client-supplied nonlinear transformation from state space to measurement space.
    // Enables EKF algorithm to compute measurement residual from state prediction.
    class EKF_MeasPredictor
    {
    public:
        using MeasPred = std::pair<Eigen::VectorXd, Eigen::MatrixXd>;
        
        // Transform state vector to measurement space, and compute corresponding 
        // Jacobian matrix of the measurement w.r.t. input state (both required by EKF).
        // Returns <measurement,Jacobian> pair.
        virtual MeasPred PredictMeasurement(const Eigen::VectorXd &state) const = 0;

        // EKF computes the measurement residual, which may need to be adjusted
        // before using for further computations.
        virtual Eigen::VectorXd NormalizeResidual(const Eigen::VectorXd &res) const = 0;
    };
}

#endif /* KALMAN_FILTER_H_ */
