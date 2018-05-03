//
// Created by Swapnil on 3/17/2018.
//

#ifndef EKFSLAM_EKFSLAMMER_H
#define EKFSLAM_EKFSLAMMER_H

#include "Eigen/Dense"
#include "Utils.h"
#include "Robot.h"

class EKFSlammer {
private:
    
    Eigen::VectorXd x; // State (location of the robot and all elements in map)
    Eigen::MatrixXd cov; // Covariance matrix for state vector

    //Stores the components of gravity in the x and y directions of the accelerometers
    //This accounts for the accelerometer not being completely level
    Eigen::Vector2d g;

    int n; //Number of stored landmark features

    //Mahalanobis distance threshold for classifying observed features as new
    double newFeatureThreshold;

    //Time step for the state update 
    const double TIME_STEP;

public:
    //EKFSlammer
    EKFSlammer(const Robot &r);

    Eigen::MatrixXd getState() const;

    Eigen::MatrixXd getCov() const;


    //getRotationMat
    //Returns 2x2 rotation matrix describing the transformation from the world reference frame to the
    //robot reference frame
    Eigen::Matrix2d getRotationMat() const;

    //getRotationMat
    //Returns 2x2 inverse rotation matrix the transformation from the robot's reference frame to the
    //world reference frame
    Eigen::Matrix2d getRotationMatInverse() const;

    //getMotionModelUncertainty
    //Returns the uncertainty in the motion model update
    Eigen::Matrix3d getMotionModelUncertainty(const control &cIn) const;

    //motionModelUpdate
    //Calculates the predicted position of the robot based on the command passed
    //TODO: Add data type for passing command.
    void motionModelUpdate(const double &deltaT, const control &controlIn);

    //setZeroPosition
    //Transforms the world reference frame to be zeroed at the new zero position (passed relative to the robot)
    void setZeroPosition(const Eigen::Vector2d &arucoMarker);

    //getTimeStep
    //Returns the elpased time between time steps
    double getTimeStep() const;


    //encoderUpdate, arucoUpdate, and kinectUpdate are all methods for the SLAM correction step
    //These three methods collect data from the respective sensors, calculate the expected observation
    //calculate the Kalman gain, and update the predicted state

    //encoderUpdate
    //Encoders return an estimated state for the robot
    //void encoderUpdate(Eigen::Matrix3d encEstPos);

    void kinectUpdate(const Eigen::VectorXd &z);

    void accelerometerUpdate(const Eigen::Vector2d &previousS,
                             const Eigen::Vector2d &gamma,
                             const Eigen::Vector2d encoder);

    void gyroUpdate(const double &previousTheta, const double &beta);

    void arucoUpdate(const Eigen::Vector2d &arucoMarker);

    void ekfCorrectionStep(const Eigen::VectorXd &kinectObstacles,
                           const Eigen::Vector2d &previousS,
                           const Eigen::Vector2d &gamma,
                           const Eigen::Vector2d &encoder,
                           const double &previousTheta,
                           const double &beta,
                           const Eigen::Vector2d &arucoMarker);

    void ekfUpdate(const control &controlIn,
                   const Eigen::VectorXd &kinectObstacles,
                   const Eigen::Vector2d &gamma,
                   const Eigen::Vector2d &encoder,
                   const double &beta,
                   const Eigen::Vector2d &arucoMarker);
};


#endif //EKFSLAM_EKFSLAMMER_H
