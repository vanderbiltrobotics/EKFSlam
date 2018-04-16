#include "EKFSlammer.h"



//EKFSlammer
//Initializes EKF Slammer. The robot's current pose is considered (0,0,0)
EKFSlammer::EKFSlammer(): x(Eigen::VectorXd::Constant(0)), cov(Eigen::Matrix2d::Constant(0))
{}

//motionModelUpdate
//Calculates the predicted position (mean and covariance) of the robot based on the command passed
//const double &theta - Current position of the robot
//const double &deltaT - Time step between updates
//const control &controlIn - Control input received from
void EKFSlammer::motionModelUpdate(const double &deltaT, const control &controlIn)
{
    //Store angular position in variable theta for readability
    double theta = x(2);

    //Ratio of velocity to angular velocity used for motion model update
    double vOmegaRatio = controlIn.v/controlIn.omega;

    //Calculate predicted mean based on motion model
    //x(0) - x position of robot
    //x(1) - y position of robot
    //x(2) - theta (angular position of robot, measured ccw from positive x)
    x(0) = x(0) + -1*vOmegaRatio*sin(theta) + vOmegaRatio*sin(theta + controlIn.omega*deltaT);
    x(1) = x(1) + vOmegaRatio*cos(theta) - vOmegaRatio*cos(theta + controlIn.omega*deltaT);
    x(2) = x(2) + controlIn.omega*deltaT;

    //Calculate Jacobian for motion model update
    //Gxt is the Jacobian for only the current pose of the robot (does not include landmark locations)
    //We do not need to calculate the update for the landmarks since the motion model does not update the
    //pose estimate for the landmarks. Therefore, there is no change in the covariance for any landmarks.

    //Note: Gxt is capital since g represents the motion model function while G represents the Jacobian for the
    //motion model function
    //Camel case convention is broken to maintain consistency with mathematical notation
    Eigen::Matrix3d Gxt = Eigen::Matrix3d::Identity();
    Gxt(0,2) = -1*(controlIn.v)/(controlIn.omega)*cos(theta)
               + (controlIn.v)/(controlIn.omega)*cos(theta + controlIn.omega*deltaT);
    Gxt(1,2) = -1*(controlIn.v)/(controlIn.omega)*sin(theta)
               + (controlIn.v)/(controlIn.omega)*sin(theta + controlIn.omega*deltaT);

    //Updating covariance matrix
    //Since the covariance does not change for the landmarks, we only need to update the portions of the
    //covariance matrix that include the pose of the robot (covXX, covXM, covMX)

    //Extract 3x3 covariance matrix for xx (pose of robot)
    //covXX is stored within columns 0-2 and rows 0-2 in cov matrix
    Eigen::Matrix3d covXX;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            covXX(i, j) = cov(i, j);
        }
    }

    //Calculated updated covariance matrix for x (pose of robot)
    //Multiply by the Jacobian
    covXX = Gxt*covXX*Gxt.transpose();

    //Add uncertainty in motion model to the covariance matrix
    covXX = covXX + getMotionModelUncertainty();

    //Extract the 3xM covariance matrix for xm (pose of robot with map)
    //covXM in the covariance matrix is within columns 3-N and rows 0-2 where N is number of rows/cols of cov matrix
    Eigen::MatrixXd covXM;
    for(int i = 0; i < 2; i++) {
        for(int j = 3; j < cov.cols(); j++) {
            covXM(i, j) = cov(i, j);
        }
    }

    //Calculated updated covariance matrix for xm
    //Multiply by the Jacobian
    covXM = Gxt*covXM;

    //Write updated covariances back to cov
    //XX Update
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            cov(i, j) = covXX(i, j);
        }
    }

    //XM Update (3-M matrix) - M is the number of landmarks stored
    for(int i = 0; i < 3; i++) {
        for(int j = 2; j < cov.cols(); j++) {
            cov(i, j) = covXM(i, j);
        }
    }

    //MX Update (M-3 matrix) - M is the number of landmarks stored
    for(int i = 0; i < cov.rows(); i++) {
        for(int j = 2; j < 3; j++) {
            cov(i, j) = covXM.transpose()(i, j);
        }
    }
}

//getMotionModelUncertainty
//Returns the uncertainty in the motion model update
Eigen::MatrixXf EKFSlammer::getMotionModelUncertainty()
{
    //TODO: Implement - Requires testing data to build error distribution
}


void EKFSlammer::ekfUpdate(const control &controlIn)
{
    double deltaT;

    //First step of the EKF update for SLAM
    //This is predicting the updated position and covariance based on only the motion model of the robot
    motionModelUpdate(deltaT, controlIn); //Calculates the estimated new position based on the motion model.

}
