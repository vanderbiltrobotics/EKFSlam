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

public:
    //EKFSlammer
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
