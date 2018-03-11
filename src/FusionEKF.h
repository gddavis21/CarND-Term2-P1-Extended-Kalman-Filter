#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class EKF_RadarMeasPredictor : public GaussianFilters::EKF_MeasPredictor 
{
public:
    EKF_RadarMeasPredictor();
    virtual MeasPred PredictMeasurement(const Eigen::VectorXd &state) const override;
    virtual Eigen::VectorXd NormalizeResidual(const Eigen::VectorXd &res) const override;

private:
    static double NormalizeHeading(double angle);
};
    
// Fuse supplied Lidar & Radar measurements with (Extended) Kalman Filter 
// algorithm to track position/velocity of object in motion.
class FusionEKF
{
public:
    FusionEKF();
    virtual ~FusionEKF();

    // Run the whole flow of the Kalman Filter from here.
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    // Query updated position/velocity
    Eigen::Vector2d GetCurrentPosition() const;
    Eigen::Vector2d GetCurrentVelocity() const;

private:
    Eigen::MatrixXd CalcStateTransition(double dt) const;
    Eigen::MatrixXd CalcProcessCovariance(double dt) const;

    double _noise_ax;
    double _noise_ay;
    bool _initialized;
    long long _prevTime;
    GaussianFilters::StateBelief _currentBelief;
    Eigen::VectorXd _control;
    Eigen::MatrixXd _ctrlTransition;
    Eigen::MatrixXd _laserMeasTransition;
    Eigen::MatrixXd _laserMeasCovariance;
    EKF_RadarMeasPredictor _radarMeasPredictor;
    Eigen::MatrixXd _radarMeasCovariance;
};

#endif /* FusionEKF_H_ */
