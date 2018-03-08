#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include <utility>
#include "Eigen/Dense"

// class KalmanFilter {
// public:

//   // state vector
//   Eigen::VectorXd x_;

//   // state covariance matrix
//   Eigen::MatrixXd P_;

//   // state transition matrix
//   Eigen::MatrixXd F_;

//   // process covariance matrix
//   Eigen::MatrixXd Q_;

//   // measurement matrix
//   Eigen::MatrixXd H_;

//   // measurement covariance matrix
//   Eigen::MatrixXd R_;

//   /**
//    * Constructor
//    */
//   KalmanFilter();

//   /**
//    * Destructor
//    */
//   virtual ~KalmanFilter();

//   /**
//    * Init Initializes Kalman filter
//    * @param x_in Initial state
//    * @param P_in Initial state covariance
//    * @param F_in Transition matrix
//    * @param H_in Measurement matrix
//    * @param R_in Measurement covariance matrix
//    * @param Q_in Process covariance matrix
//    */
//   void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
//       Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

//   /**
//    * Prediction Predicts the state and the state covariance
//    * using the process model
//    * @param delta_T Time between k and k+1 in s
//    */
//   void Predict();

//   /**
//    * Updates the state by using standard Kalman Filter equations
//    * @param z The measurement at k+1
//    */
//   void Update(const Eigen::VectorXd &z);

//   /**
//    * Updates the state by using Extended Kalman Filter equations
//    * @param z The measurement at k+1
//    */
//   void UpdateEKF(const Eigen::VectorXd &z);

// };

struct StateBelief
{
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
};

class EKF_MeasPredictor
{
public:
    using MeasPred = std::pair<Eigen::VectorXd, Eigen::MatrixXd>;
    virtual MeasPred PredictMeasurement(const Eigen::VectorXd &state) const = 0;
    virtual Eigen::VectorXd NormalizeResidual(const Eigen::VectorXd &res) const = 0;
};

class KalmanFilter
{
public:
    // standard Kalman Filter
    static StateBelief KF(
        const StateBelief &priorBelief, 
        const Eigen::VectorXd &control,
        const Eigen::MatrixXd &stateTransition,
        const Eigen::MatrixXd &controlTransition,
        const Eigen::MatrixXd &processCovariance,
        const Eigen::VectorXd &measurement,
        const Eigen::MatrixXd &measTransition,
        const Eigen::MatrixXd &measCovariance);

    // Extended Kalman Filter with Linear Prediction (nonlinear measurement update)
    static StateBelief EKF_LinearPred(
        const StateBelief &priorBelief,
        const Eigen::VectorXd &control,
        const Eigen::MatrixXd &stateTransition,
        const Eigen::MatrixXd &controlTransition,
        const Eigen::MatrixXd &processCovariance,
        const Eigen::VectorXd &measurement,
        const EKF_MeasPredictor &measPredictor,
        const Eigen::MatrixXd &measCovariance);
};

#endif /* KALMAN_FILTER_H_ */
