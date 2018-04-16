//
// Created by Swapnil on 3/17/2018.
//

#ifndef EKFSLAM_EKFSLAMMER_H
#define EKFSLAM_EKFSLAMMER_H

#include "Eigen/Dense"
#include "Utils.h"

class EKFSlammer {
private:
    Eigen::VectorXd x; //Stores current state (pose and location of all elements in map)
    Eigen::MatrixXd cov; //Stores covariance matrix for state vector

public:
    //EKFSlammer
    //Initializes EKF Slammer. The robot's current pose is considered (0,0,0)
    EKFSlammer();

    //motionModelUpdate
    //Calculates the predicted position of the robot based on the command passed
    //TODO: Add data type for passing command.
    void motionModelUpdate(const double &deltaT, const control &controlIn);

    //getMotionModelUncertainty
    //Returns the uncertainty in the motion model update
    Eigen::MatrixXf getMotionModelUncertainty();

    void ekfUpdate(const control &controlIn);
};


#endif //EKFSLAM_EKFSLAMMER_H
