#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>
#include <stdexcept>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using namespace GaussianFilters;

static const double PI = acos(-1.0);

FusionEKF::FusionEKF() 
{
    _initialized = false;
    _prevTime = 0;
    _noise_ax = _noise_ay = 9.0;

    // fixed measurement transition matrix for laser measurements
    _laserMeasTransition = MatrixXd(2, 4);
    _laserMeasTransition << 
        1, 0, 0, 0,
        0, 1, 0, 0;
    
    // fixed measurement covariance matrix for laser measurements
    _laserMeasCovariance = MatrixXd(2,2);
    _laserMeasCovariance << 
        0.0225, 0.    ,
        0.    , 0.0225;
    
    // fixed measurement covariance matrix for radar measurements
    _radarMeasCovariance = MatrixXd(3,3);
    _radarMeasCovariance <<
        0.09, 0.    , 0.  ,
        0.  , 0.0009, 0.  ,
        0.  , 0.    , 0.09;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
    long long measTime = measurement_pack.timestamp_;
    MeasurementPackage::SensorType sensorType = measurement_pack.sensor_type_;
    VectorXd measurement = measurement_pack.raw_measurements_;
    
    /*****************************************************************************
    *  State Initialization
    ****************************************************************************/
    if (!_initialized) 
    {
        // initialize state vector
        double x, y;

        switch (sensorType)
        {
            case MeasurementPackage::LASER:
                // Laser measurement already in cartesian coordinates
                x = measurement(0);
                y = measurement(1);
                break;

            case MeasurementPackage::RADAR:
            {
                // Convert radar measurement from polar to cartesian coordinates
                double range = measurement(0);
                double heading = measurement(1);
                x = range*cos(heading);
                y = range*sin(heading);
                break;
            }

            default:
                throw std::invalid_argument("unrecognized sensor type");
        }

        _currentBelief.state = VectorXd(4);
        _currentBelief.state << x, y, 1, 1;
        
        // initialize covariance
        _currentBelief.covariance = MatrixXd(4,4);
        _currentBelief.covariance <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

        // done initializing, no need to predict or update
        _prevTime = measTime;
        _initialized = true;
        return;
    }

    /*****************************************************************************
    *  State Prediction & Update
    ****************************************************************************/

    // compute the time elapsed between the current and previous measurements
    double dt = (measTime - _prevTime) / 1000000.0;	//dt - expressed in seconds
    _prevTime = measTime;

    MatrixXd stateTransition = CalcStateTransition(dt);
    MatrixXd processCovariance = CalcProcessCovariance(dt);

    // dummy control vector (not using control vector in this project)
    static VectorXd control = VectorXd::Zero(4);
    static MatrixXd ctrlTransition = MatrixXd::Zero(4,4);

    // choose update algorithm depending on sensor type
    switch (sensorType)
    {
        // For laser measurements, update state belief with the standard Kalman 
        // Filter algorithm (linear state prediction, linear measurement update).
        case MeasurementPackage::LASER:
            _currentBelief = KalmanFilter(
                _currentBelief,
                control,
                stateTransition,
                ctrlTransition,
                processCovariance,
                measurement,
                _laserMeasTransition,
                _laserMeasCovariance);
            break;
        
        // For radar measurements, update state belief with a simplified version
        // of the Extended Kalman Filter algorithm (linear state prediction, 
        // nonlinear measurement update). Have to use EKF because of the nonlinear
        // relationship between (polar) radar measurements and (rectangular) state
        // vector.
        case MeasurementPackage::RADAR:
            // Radar updates
            _currentBelief = ExtendedKalmanFilter_LinearPred(
                _currentBelief,
                control,
                stateTransition,
                ctrlTransition,
                processCovariance,
                measurement,
                _radarMeasPredictor,
                _radarMeasCovariance);
            break;

        default:
            throw std::invalid_argument("unrecognized sensor type");
    }
}

Vector2d FusionEKF::GetCurrentPosition() const
{
    return Vector2d(_currentBelief.state(0), _currentBelief.state(1));
}

Vector2d FusionEKF::GetCurrentVelocity() const
{
    return Vector2d(_currentBelief.state(2), _currentBelief.state(3));
}

// compute state transition matrix, given current time delta
MatrixXd FusionEKF::CalcStateTransition(double dt) const
{
    // integrate elapsed time into state transition matrix
    MatrixXd A(4,4);
    A <<  
        1,  0, dt,  0,
        0,  1,  0, dt,
        0,  0,  1,  0,
        0,  0,  0,  1;
        
    return A;
}

// compute process covariance matrix, given current time delta
MatrixXd FusionEKF::CalcProcessCovariance(double dt) const
{
    double dt2 = dt*dt;     // dt^2
    double dt3 = dt2*dt;    // dt^3
    double dt4 = dt3*dt;    // dt^4

    double nx = _noise_ax;
    double ny = _noise_ay;

    MatrixXd R(4,4);
    R << 
        nx*dt4/4,        0, nx*dt3/2,        0,
               0, ny*dt4/4,        0, ny*dt3/2,
        nx*dt3/2,        0,   nx*dt2,        0,
               0, ny*dt3/2,        0,   ny*dt2;

    return R;
}

EKF_RadarMeasPredictor::EKF_RadarMeasPredictor() {}

// Transform position/velocity state vector into radar measurement space (range/heading/range-rate).
// Also compute Jacobian matrix of transformed radar vector w.r.t. input state.
// Return radar/Jacobian pair.
EKF_MeasPredictor::MeasPred EKF_RadarMeasPredictor::PredictMeasurement(const VectorXd &state) const
{
    // unpack state vector
    double x = state(0);   // x position
    double y = state(1);   // y position
    double u = state(2);   // x velocity
    double v = state(3);   // y velocity
    
    // compute radar range
    double range = sqrt(x*x + y*y);
    
    if (range < 0.0001)
    {
        cout << "Divide-by-zero error" << endl;
        return make_pair(
            VectorXd::Zero(3),
            MatrixXd::Zero(3,4));
    }
    
    // compute radar heading angle & range-rate
    double heading = NormalizeHeading(atan2(y, x));
    double range_rate = (x*u + y*v) / range;
    
    VectorXd meas(3);
    meas << range, heading, range_rate;
    
    // compute Jacobian for linearized radar measurement update
    double r = range;
    double r2 = r*r;
    double r3 = r2*r;
    double n = u*y - v*x;
    
    MatrixXd jacobian(3,4);
    jacobian << 
        x/r,     y/r,   0,   0,
        -y/r2,    x/r2,   0,   0,
        y*n/r3, -x*n/r3, x/r, y/r;

    return make_pair(meas, jacobian);
}

// normalize the residual heading angle
VectorXd EKF_RadarMeasPredictor::NormalizeResidual(const VectorXd &res) const
{
    VectorXd normRes(3);
    normRes << res(0), NormalizeHeading(res(1)), res(2);
    return normRes;
}

// ensure angle stays within [-PI,PI) range
double EKF_RadarMeasPredictor::NormalizeHeading(double angle)
{
    return fmod(angle + PI, 2*PI) - PI;
}
