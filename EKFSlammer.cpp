#include <d2d1_1helper.h>
#include "EKFSlammer.h"

/*TODO LIST
    todo - Add handling for angle rollover
    todo - Determine how to add motion model and measurement noise
    todo - Add ML approximation for Kinect data association
    todo - Determine velocity term for accelerometer update
    todo - Implement time step
    todo - Implement coordinate transformation for beginning of algorithm execution
 */


//EKFSlammer
//Initializes EKF Slammer. The robot's current pose is considered (0,0,0)
EKFSlammer::EKFSlammer(): x(Eigen::VectorXd::Constant(0)), cov(Eigen::Matrix2d::Constant(0)),
                          g(Eigen::Matrix2d::Constant(0)), n(0)
{}

//getRotationMat
//Returns 2x2 rotation matrix describing the transformation from the world reference frame to the
//robot reference frame
Eigen::Matrix2d EKFSlammer::getRotationMat()
{
    Eigen::Matrix2d rotation;
    rotation(0,0) = cos(x(2));
    rotation(0,1) = -1*sin(x(2));
    rotation(1,0) = -1*rotation(0,1);
    rotation(1,1) = cos(x(2));

    return rotation;
}
//getRotationMat
//Returns 2x2 inverse rotation matrix the transformation from the robot's reference frame to the
//world reference frame
Eigen::Matrix2d EKFSlammer::getRotationMatInverse()
{
    Eigen::Matrix2d rotation;
    rotation(0,0) = cos(x(2));
    rotation(0,1) = sin(x(2));
    rotation(1,0) = -1*rotation(0,1);
    rotation(1,1) = cos(x(2));

    return rotation;
}


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


//kinectUpdate
//Kinect returns the distance and angle (relative to the robot) of detected obstacles in the environment
//Calculates the updated position of the robot based on the measurements
void EKFSlammer::kinectUpdate(Eigen::VectorXd &z)
{
    int obstacleIndex; //Stores the index of the obstacle that is currently being operated on.

    Eigen::MatrixXd Q;



    for(int i = 0; i < z.rows()/2; i++)
    {
        //TODO Implement ML approximation to associate detected obstacle to stored obstacle
        //TODO Implement adding new obstacles

//        //Transforming the coordinate system of the observed obstacle from (r,theta) to (x,y)
//        double observedObstX = x(0) + zt(0)*cos(zt(1) + x(2));
//        double observedObstY = x(1) + zt(0)*sin(zt(1) + x(2));

        //delta stores the difference in expected position of landmark and expected position of robot
        Eigen::VectorXd delta;
        delta << (x(2*i) - x(0)),
                 (x(2*i+1) - x(1));

        double q = delta.dot(delta);

        //h stores the the predicted observation for the given landmark.
        Eigen::VectorXd h;
        h << sqrt(q),
                (atan2(delta(1),delta(0)) - x(2));

        //H is the Jacobian of the h - Jacobian of the predicted sensor measurement
        //H is a 2x5 matrix that gets mapped to a higher dimensional matrix. The computation of the
        //jacobian and mapping to the higher dimension is taking place in one step by directly assigning
        //the values to individual matrix indices.
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,cov.rows());
        H(0,0) = -1*delta(0)/h(0);
        H(0,1) = -1*delta(1)/h(0);
        //H(0,2) = 0; In here just for readability
        H(0,3+2*i) = delta(0)/h(0);
        H(0,4+2*i) = delta(1)/h(0);

        H(1,0) = delta(1)/q;
        H(1,1) = -1*delta(0)/q;
        H(1,2) = -1*q;
        H(1,3+2*i) = -1*delta(1)/q;
        H(1,4+2*i) = delta(0)/q;

        //Kalman gain
        Eigen::MatrixXd K;

        //Measurement Update occurs here
        K = cov*H.transpose()*(H*cov*H.transpose()+Q).inverse(); //Calculate Kalman gain
        x = x + K*(z-h); //Update state estimate
        Eigen::MatrixXd KH = K*H;
        cov = (Eigen::MatrixXd::Identity(KH.rows(),KH.cols()) - KH)*cov; //Update covariance matrix

    }
}

//accelerometerUpdate
//Accelerometer returns the acceleration in the x and y directions. This is used to calcalate the "observed"
//position update, which is compared to the position calculated by the motion model update
void EKFSlammer::accelerometerUpdate(Eigen::Vector2d &previousS, Eigen::Vector2d &gamma)
{
    double timeStep = getTimeStep();
    Eigen::MatrixXd Q;

    //Calculate the planar acceleration in the world's reference frame by transforming the vector by the inverse
    //rotation matrix and subtracting the components of gravity.,
    Eigen::Vector2d a = getRotationMatInverse()*gamma - g;

    //Calculates the position of the robot based on the previous time step and the acceleration of the robot
    //TODO Ask Dr. Peters about including robot velocity here. Perhaps use encoders or control input to get velocity?
    Eigen::Vector2d s = previousS + 0.5*pow(timeStep,2)*a;

    //h stores the predicted position of the robot based on the motion model update
    Eigen::Vector2d h;
    h << x(0),
            x(1);

    //H is the Jacobian of the h - Jacobian of the predicted sensor measurement
    //H is a 2x5 matrix that gets mapped to a higher dimensional matrix. The computation of the
    //jacobian and mapping to the higher dimension is taking place in one step by directly assigning
    //the values to individual matrix indices.
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,cov.rows());
    H(0,0) = 1;
    H(1,1) = 1;

    //Kalman gain
    Eigen::MatrixXd K;

    //Measurement Update occurs here
    K = cov*H.transpose()*(H*cov*H.transpose()+Q).inverse(); //Calculate Kalman gain
    x = x + K*(a-h); //Update state estimate
    Eigen::MatrixXd KH = K*H;
    cov = (Eigen::MatrixXd::Identity(KH.rows(),KH.cols()) - KH)*cov; //Update covariance matrix
}

//accelerometerUpdate
//Accelerometer returns the acceleration in the x and y directions. This is used to calcalate the "observed"
//position update, which is compared to the position calculated by the motion model update
void EKFSlammer::gyroUpdate(double &previousTheta, double &beta)
{
    //Stores deltaT
    double timeStep = getTimeStep();

    Eigen::MatrixXd Q;

    //Calculates the orientation of the robot based on the angular velocity and previous orientation
    double theta = previousTheta + beta*timeStep;

    //h stores the predicted orientation of the robot based on the motion model update
    double h = x(2);

    //H is the Jacobian of the h - Jacobian of the predicted sensor measurement
    //H is a 2x5 matrix that gets mapped to a higher dimensional matrix. The computation of the
    //jacobian and mapping to the higher dimension is taking place in one step by directly assigning
    //the values to individual matrix indices.
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,cov.rows());
    H(3,3) = 1;

    //Kalman gain
    Eigen::MatrixXd K;

    //Measurement Update occurs here
    K = cov*H.transpose()*(H*cov*H.transpose()+Q).inverse(); //Calculate Kalman gain
    x = x + K*(theta-h); //Update state estimate
    Eigen::MatrixXd KH = K*H;
    cov = (Eigen::MatrixXd::Identity(KH.rows(),KH.cols()) - KH)*cov; //Update covariance matrix
}

void EKFSlammer::arucoUpdate(Eigen::Vector2d &arucoMarker)
{
    int obstacleIndex; //Stores the index of the obstacle that is currently being operated on.

    Eigen::MatrixXd Q;
        //delta stores the difference in expected position of marker (0,0) and expected position of robot
        Eigen::VectorXd delta;
        delta << (0 - x(0)),
                (0 - x(1));

        double q = delta.dot(delta);

        //h stores the the predicted observation for the given landmark.
        Eigen::VectorXd h;
        h << sqrt(q),
                (atan2(delta(1),delta(0)) - x(2));

        //Location of the AruCo marker, which is known to be (0,0)
        Eigen::Vector2d z;
        z << 0,0;

        //H is the Jacobian of the h - Jacobian of the predicted sensor measurement
        //H is a 2x5 matrix that gets mapped to a higher dimensional matrix. The computation of the
        //jacobian and mapping to the higher dimension is taking place in one step by directly assigning
        //the values to individual matrix indices.
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,cov.rows());
        H(0,0) = -1*delta(0)/h(0);
        H(0,1) = -1*delta(1)/h(0);

        H(1,0) = delta(1)/q;
        H(1,1) = -1*delta(0)/q;


        //Kalman gain
        Eigen::MatrixXd K;

        //Measurement Update occurs here
        K = cov*H.transpose()*(H*cov*H.transpose()+Q).inverse(); //Calculate Kalman gain
        x = x + K*(z-h); //Update state estimate
        Eigen::MatrixXd KH = K*H;
        cov = (Eigen::MatrixXd::Identity(KH.rows(),KH.cols()) - KH)*cov; //Update covariance matrix

    }
}

void EKFSlammer::ekfCorrectionStep(Eigen::VectorXd &kinectObstacles,
                                   Eigen::Vector2d &previousS, Eigen::Vector2d &gamma,
                                   double &previousTheta, double &beta
                                   Eigen::Vector2d &arucoMarker)
{
    //Update based on the obstacles detected by the Kinect. The Kinect returns the range and bearing of
    //obstacles in the environment.
    EKFSlammer::kinectUpdate(kinectObstacles);

    //Update based on the acceleration measured by the accelerometer.
    accelerometerUpdate(previousS, gamma);

    //Update based on the angular velocity measured by the gyro.
    gyroUpdate(previousTheta, beta);

    arucoUpdate(arucoMarker);

}

//TODO Handle angle updates correctly: incorporate rollover
void EKFSlammer::ekfUpdate(const control &controlIn,
                           Eigen::VectorXd &kinectObstacles,
                           Eigen::Vector2d &gamma,
                           double &beta,
                           Eigen::Vector2d &arucoMarker)
{
    double deltaT;

    //Stores the previous position vector of the robot, needed for the acclerometer
    Eigen::Vector3d previousS;
    previousS << x(0),x(1);

    //Stores the previous position angle of the robot, needed for the gyro
    double previousTheta = x(2);

    //First step of the EKF update for SLAM
    //This is predicting the updated position and covariance based on only the motion model of the robot
    motionModelUpdate(deltaT, controlIn); //Calculates the estimated new position based on the motion model.

    //Second step of EKF SLAM.
    //This is updating the prediction based on the data from the sensors. The observed values from the sensors
    //are compared to the predicted value from the previous step. Depending on error of the motion model
    //and the error of the sensor model, the update is weighted towards one or the other.
    ekfCorrectionStep();
}
