//
// Created by Swapnil on 3/17/2018.
//

#ifndef EKFSLAM_EKFSLAMMER_H
#define EKFSLAM_EKFSLAMMER_H

#include "Eigen/Dense"
#include "Utils.h"

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


public:
    //EKFSlammer
    EKFSlammer();

    //getRotationMat
    //Returns 2x2 rotation matrix describing the transformation from the world reference frame to the
    //robot reference frame
    Eigen::Matrix2d getRotationMat();

    //getRotationMat
    //Returns 2x2 inverse rotation matrix the transformation from the robot's reference frame to the
    //world reference frame
    Eigen::Matrix2d getRotationMatInverse();

    //getMotionModelUncertainty
    //Returns the uncertainty in the motion model update
    Eigen::MatrixXf getMotionModelUncertainty();

    //setZeroPosition
    //Transforms the world reference frame to be zeroed at the new zero position (passed relative to the robot)
    void setZeroPosition(Eigen::Vector2d &arucoMarker);

    //getTimeStep
    //Returns the elpased time between time steps
    int getTimeStep();

    //motionModelUpdate
    //Calculates the predicted position of the robot based on the command passed
    //TODO: Add data type for passing command.
    void motionModelUpdate(const double &deltaT, const control &controlIn);


    //encoderUpdate, arucoUpdate, and kinectUpdate are all methods for the SLAM correction step
    //These three methods collect data from the respective sensors, calculate the expected observation
    //calculate the Kalman gain, and update the predicted state

    //encoderUpdate
    //Encoders return an estimated state for the robot
    //void encoderUpdate(Eigen::Matrix3d encEstPos);

    void kinectUpdate(Eigen::VectorXd &z);

    void accelerometerUpdate(Eigen::Vector2d &previousS, Eigen::Vector2d &gamma, Eigen::Vector2d encoder);

    void gyroUpdate(double &previousTheta, double &beta);

    void arucoUpdate(Eigen::Vector2d &arucoMarker);

    void ekfCorrectionStep();

    void ekfUpdate(const control &controlIn,
                            Eigen::VectorXd &kinectObstacles,
                            Eigen::Vector2d &gamma,
                            double &beta,
                            Eigen::Vector2d &arucoMarker);
};


#endif //EKFSLAM_EKFSLAMMER_H
