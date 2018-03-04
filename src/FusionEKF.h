#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  // /**
  // * Kalman Filter update and prediction math lives in here.
  // */
  // KalmanFilter ekf_;
  Eigen::Vector2d GetCurrentPosition() const;
  Eigen::Vector2d GetCurrentVelocity() const;
  

private:
  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  
    static Eigen::MatrixXd StateTransition(double dt);
    static Eigen::MatrixXd ProcessCovariance(double dt, double nx, double ny);
    
    bool _initialized;
    long long _prevTime;
    StateBelief _currentBelief;
    Eigen::VectorXd _control;
    Eigen::MatrixXd _ctrlTransition;
    Eigen::MatrixXd _measTransition;
    Eigen::MatrixXd _measCovariance_Laser;
    Eigen::MatrixXd _measCovariance_Radar;
};

#endif /* FusionEKF_H_ */
