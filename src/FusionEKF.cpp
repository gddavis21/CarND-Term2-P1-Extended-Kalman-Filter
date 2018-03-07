#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

static const double PI = acos(-1.0);

class EKF_RadarMeasPredictor : public EKF_MeasPredictor
{
public:
    virtual MeasPred PredictMeasurement(const VectorXd &state) const override
    {
        double x = state(0);   // x position
        double y = state(1);   // y position
        double u = state(2);   // x velocity
        double v = state(3);   // y velocity
        
        double range = sqrt(x*x + y*y);
        
        if (range < 0.0001)
        {
            cout << "Divide-by-zero error" << endl;
            return make_pair(
                VectorXd::Zero(3),
                MatrixXd::Zero(3,4));
        }
        
        double heading = NormalizeAngle(atan2(y, x));
        double range_rate = (x*u + y*v) / range;
        
        VectorXd meas(3);
        meas << range, heading, range_rate;
        
        double r = range;
        double r2 = r*r;
        double r3 = r2*r;
        double n = u*y - v*x;
        
        // compute Jacobian of radar 
        MatrixXd jacobian(3,4);
        jacobian << 
            x/r,     y/r,   0,   0,
            -y/r2,    x/r2,   0,   0,
            y*n/r3, -x*n/r3, x/r, y/r;

        return make_pair(meas, jacobian);
    }

    virtual VectorXd NormalizeResidual(const VectorXd &res) const override
    {
        VectorXd normRes(3);
        normRes << res(0), NormalizeAngle(res(1)), res(2);
        return normRes;
    }

private:
    static double NormalizeAngle(double a)
    {
        return fmod(a + PI) - PI;
    }
};

// VectorXd StateToRadar(const VectorXd &state) 
// {
//     double x = state(0);   // x position
//     double y = state(1);   // y position
//     double u = state(2);   // x velocity
//     double v = state(3);   // y velocity
    
//     double range = sqrt(x*x + y*y);
    
//     if (range < 0.0001)
//         return VectorXd::Zero(3);
    
//     double heading = atan2(y, x);
//     double range_rate = (x*u + y*v) / range;
    
//     while (heading > PI)
//         heading -= 2*PI;
    
//     while (heading < -PI)
//         heading += 2*PI;
    
//     VectorXd radar(3);
//     radar << range, heading, range_rate;
//     return radar;
// }

// MatrixXd StateToRadarJacobian(const VectorXd &state)
// {
//     // recover state parameters
//     double x = state(0);   // x position
//     double y = state(1);   // y position
//     double u = state(2);   // x velocity
//     double v = state(3);   // y velocity

//     double r = sqrt(x*x + y*y);
//     double r2 = r*r;
//     double r3 = r2*r;
//     double n = u*y - v*x;
	
//     // check for division by zero
//     if (r < 0.0001)
//     {
//         cout << "Divide-by-zero error" << endl;
//         return MatrixXd::Zero(3,4);
//     }

//     // compute Jacobian of radar 
//     MatrixXd jacobian(3,4);
//     jacobian << 
//            x/r,     y/r,   0,   0,
//          -y/r2,    x/r2,   0,   0,
//         y*n/r3, -x*n/r3, x/r, y/r;

//     return jacobian;
// }

/*
 * Constructor.
 */
FusionEKF::FusionEKF() 
{
    _initialized = false;
    _prevTime = 0;
    _noise_ax = _noise_ay = 9;

    // not tracking control vector in this project
    _control = VectorXd::Zero(4);
    _ctrlTransition = MatrixXd::Zero(4,4);

    _measTransition = MatrixXd(2, 4);
    _measTransition << 
        1, 0, 0, 0,
        0, 1, 0, 0;
    
    _measCovariance_Laser = MatrixXd(2,2);
    _measCovariance_Laser << 
        0.0225, 0.    ,
        0.    , 0.0225;
    
    _measCovariance_Radar = MatrixXd(3,3);
    _measCovariance_Radar <<
        0.09, 0.    , 0.  ,
        0.  , 0.0009, 0.  ,
        0.  , 0.    , 0.09;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
    long long measTime = measurement_pack.timestamp_;
    MeasurementPackage::SensorType sensorType = measurement_pack.sensor_type_;
    VectorXd measurement = measurement_pack.raw_measurements_;
    
    /*****************************************************************************
    *  Initialization
    ****************************************************************************/
    if (!_initialized) 
    {
        // // first measurement
        // cout << "EKF: " << endl;
        // ekf_.x_ = VectorXd(4);
        // ekf_.x_ << 1, 1, 1, 1;
        
        // initialize state
        double x, y;

        if (sensorType == MeasurementPackage::RADAR) 
        {
            // Convert radar from polar to cartesian coordinates
            double range = measurement(0);
            double heading = measurement(1);
            
            x = range*cos(heading);
            y = range*sin(heading);
        }
        else if (sensorType == MeasurementPackage::LASER) 
        {
            // Laser already in cartesian coordinates
            x = measurement(0);
            y = measurement(1);
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
    *  Prediction
    ****************************************************************************/

    /**
    TODO:
    * Update the state transition matrix F according to the new elapsed time.
    - Time is measured in seconds.
    * Update the process noise covariance matrix.
    * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    */

    //compute the time elapsed between the current and previous measurements
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    /**
    TODO:
    * Use the sensor type to perform the update step.
    * Update the state and covariance matrices.
    */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
        EKF_RadarMeasPredictor radarMeasPredictor;

        // Radar updates
        _currentBelief = LinearPredExtendedKalmanFilter(
            _currentBelief,
            _control,
            measurement_pack.raw_measurements_,
            StateTransition(dt),
            _ctrlTransition,
            ProcessCovariance(dt, _noise_ax, _noise_ay),
            radarMeasPredictor,
            _measCovariance_Radar);
    } 
    else 
    {
        // Laser updates
        _currentBelief = KalmanFilter(
            _currentBelief,
            _control,
            measurement_pack.raw_measurements_,
            StateTransition(dt),
            _ctrlTransition,
            ProcessCovariance(dt, _noise_ax, _noise_ay),
            _measTransition,
            _measCovariance_Laser);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}

Eigen::Vector2d FusionEKF::GetPosition() const
{
    return Vector2d(_currentBelief.state(0), _currentBelief.state(1));
}

Eigen::Vector2d FusionEKF::GetVelocity() const
{
    return Vector2d(_currentBelief.state(2), _currentBelief.state(3));
}

Eigen::MatrixXd FusionEKF::StateTransition(double dt)
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

Eigen::MatrixXd FusionEKF::ProcessCovariance(
    double dt,  // elapsed time
    double nx,  // x-acceleration noise variance
    double ny)  // y-acceleration noise variance
{
    double dt2 = dt*dt;     // dt^2
    double dt3 = dt2*dt;    // dt^3
    double dt4 = dt3*dt;    // dt^4

    MatrixXd R(4,4);
    R << 
        nx*dt4/4,        0, nx*dt3/2,        0,
               0, ny*dt4/4,        0, ny*dt3/2,
        nx*dt3/2,        0,   nx*dt2,        0,
               0, ny*dt3/2,        0,   ny*dt2;

    return R;
}
