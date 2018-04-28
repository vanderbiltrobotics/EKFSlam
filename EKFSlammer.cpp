//#include <d2d1_1helper.h>
#include "EKFSlammer.h"


/*TODO LIST
    todo - Add handling for angle rollover
    todo - Determine how to add motion model and measurement noise
    todo - Determine velocity term for accelerometer update
    todo - Implement coordinate transformation for beginning of algorithm execution
 */


//EKFSlammer
//Initializes EKF Slammer. The robot's current pose is considered (0,0,0)
EKFSlammer::EKFSlammer() : x(Eigen::Vector3d::Constant(0)),
                           cov(Eigen::Matrix2d::Constant(0)),
                           g(Eigen::Vector2d::Constant(0)),
                           n(0)
{
    cov(0,0) = std::numeric_limits<double>::max();
    cov(1,1) = std::numeric_limits<double>::max();
    cov(2,2) = std::numeric_limits<double>::max();

}


double EKFSlammer::getTimeStep()
{
    return 0.020; //Time Step in seconds. Constant for the purpose of the simulation
}

//getRotationMat
//Returns 2x2 rotation matrix describing the transformation from the world reference frame to the
//robot reference frame
Eigen::Matrix2d EKFSlammer::getRotationMat()
{
    // declare these to avoid having to recalculate them
    double costerm = cos(x(2));
    double sinterm = sin(x(2));

    Eigen::Matrix2d rotation;
    rotation << costerm, -sinterm,
                sinterm, costerm;

    return rotation;
}
//getRotationMat
//Returns 2x2 inverse rotation matrix the transformation from the robot's reference frame to the
//world reference frame
Eigen::Matrix2d EKFSlammer::getRotationMatInverse()
{
    // since you insist on defining this method... :P
    return getRotationMat().inverse();
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
    x(0) += -vOmegaRatio*sin(theta) + vOmegaRatio*sin(theta + controlIn.omega*deltaT);
    x(1) += vOmegaRatio*cos(theta) - vOmegaRatio*cos(theta + controlIn.omega*deltaT);
    x(2) += controlIn.omega*deltaT;

    //Calculate Jacobian for motion model update
    //Gxt is the Jacobian for only the current pose of the robot (does not include landmark locations)
    //We do not need to calculate the update for the landmarks since the motion model does not update the
    //pose estimate for the landmarks. Therefore, there is no change in the covariance for any landmarks.

    //Note: Gxt is capital since g represents the motion model function while G represents the Jacobian for the
    //motion model function
    Eigen::Matrix3d Gxt = Eigen::Matrix3d::Identity();
    Gxt(0,2) = -(controlIn.v)/(controlIn.omega)*cos(theta)
               + (controlIn.v)/(controlIn.omega)*cos(theta + controlIn.omega*deltaT);
    Gxt(1,2) = -(controlIn.v)/(controlIn.omega)*sin(theta)
               + (controlIn.v)/(controlIn.omega)*sin(theta + controlIn.omega*deltaT);

    //Updating covariance matrix
    //Since the covariance does not change for the landmarks, we only need to update the portions of the
    //covariance matrix that include the pose of the robot (covXX, covXM, covMX)

    //Extract 3x3 covariance matrix for xx (pose of robot)
    //covXX is stored within columns 0-2 and rows 0-2 in cov matrix
    Eigen::Matrix3d covXX = cov.block<3,3>(0, 0);

    //Calculated updated covariance matrix for x (pose of robot)
    //Multiply by the Jacobian
    covXX = Gxt*covXX*Gxt.transpose();

    //Add uncertainty in motion model to the covariance matrix
    covXX += getMotionModelUncertainty();

    //Extract the 3xM covariance matrix for xm (pose of robot with map)
    //covXM in the covariance matrix is within columns 3-N and rows 0-2 where N is number of rows/cols of cov matrix
    Eigen::MatrixXd covXM = cov.block(0, 3, 3, 3-n);

    //Calculated updated covariance matrix for xm
    //Multiply by the Jacobian
    covXM = Gxt*covXM;

    //Write updated covariances back to cov
    //XX Update
    cov.block<3,3>(0, 0) = covXX;

    //XM Update (3-n matrix) - n is the number of landmarks stored
    cov.block(0, 3, 3, 3-n) = covXM;
    // for(int i = 0; i < 3; i++) {
    //     for(int j = 2; j < cov.cols(); j++) {
    //         cov(i, j) = covXM(i, j);
    //     }
    // }

    //MX Update (n-3 matrix) - n is the number of landmarks stored
    cov.block(3, 0, 3-n, 3) = covXM.transpose();
    // for(int i = 0; i < cov.rows(); i++) {
    //     for(int j = 2; j < 3; j++) {
    //         cov(i, j) = covXM.transpose()(i, j);
    //     }
    // }
}

//getMotionModelUncertainty
//Returns the uncertainty in the motion model update
Eigen::MatrixXd EKFSlammer::getMotionModelUncertainty()
{
    Eigen::Matrix3d error = Eigen::MatrixXd::Constant(3, 3, 0);
    // provisional, 3x3 matrix of ones
    return Eigen::MatrixXd::Constant(3, 3, 1);
}


//kinectUpdate
//Kinect returns the distance and angle (relative to the robot) of detected obstacles in the environment
//Calculates the updated position of the robot based on the measurements
void EKFSlammer::kinectUpdate(const Eigen::VectorXd &z)
{
    int obstacleIndex; //Stores the index of the obstacle that is currently being operated on.

    Eigen::MatrixXd Q;



    for(int i = 0; i < z.rows()/2; i++)
    {
        //zCur stores the current measurement being operated on
        Eigen::Vector2d zCur;
        zCur << (z(2*i)),
                (z(2*i+1));
        //First step of the update is to use maximum likelihood approximation to determine associate the
        //measurement of the obstacle to a previously detected obstacle.

        //Maximum likelihood approximation starts by creating the hypothesis of a new landmark.
        //The location of the landmark is calculated by transforming the range-bearing measurement to the
        //world reference frame
        double newObstX = x(0) + zCur(0)*cos(zCur(1) + x(2));
        double newObstY = x(1) + zCur(0)*sin(zCur(1) + x(2));
        int newObstInd = n+1; //Index of the new obstacle is n+1

        //h stores the the predicted observation for the given landmark (range bearing).
        Eigen::VectorXd h;

        //H is the Jacobian of the h - Jacobian of the predicted sensor measurement
        //H is a 2x5 matrix that gets mapped to a higher dimensional matrix.
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,cov.rows());

        //Psi stores the sensor error term for the Kalman gain
        Eigen::MatrixXd psi = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());
        //Pi stores the Mahalanobis distance between calculated for ML approximation
        //Pi is initialized to the new feature threshold.
        double pi = newFeatureThreshold;

        //Comparing the observed measurement to all stored obstacles.
        //The Mahalanobis distance between the measurement and existing obstacles is calculated.
        //The obstacle index is stored for the obstacle with minimum distance.
        for(int j = 0; j < n; j++)
        {
            //delta stores the difference in expected position of landmark n and expected position of robot
            Eigen::VectorXd delta;
            //Add three to index to skip over (x,y,theta)
            delta << (x(2*j+3) - x(0)),
                    (x(2*j+1+3) - x(1));

            //Calculating distance between expected position of landmark and expected position of robot (r^2)
            double q = delta.dot(delta);

            //h stores the the predicted observation for the given landmark (range bearing).
            Eigen::Vector2d hTemp;
            hTemp << sqrt(q),
                    (atan2(delta(1),delta(0)) - x(2));

            //H is the Jacobian of the h - Jacobian of the predicted sensor measurement
            //H is a 2x5 matrix that gets mapped to a higher dimensional matrix. The computation of the
            //jacobian and mapping to the higher dimension is taking place in one step by directly assigning
            //the values to individual matrix indices.
            //This is temporary because H is calculated for each stored obstacle. One one matrix H is stored
            Eigen::MatrixXd HTemp = Eigen::MatrixXd::Zero(2,cov.rows());
            HTemp(0,0) = -1*delta(0)/h(0);
            HTemp(0,1) = -1*delta(1)/h(0);
            //H(0,2) = 0; In here just for readability
            HTemp(0,3+2*i) = delta(0)/h(0);
            HTemp(0,4+2*i) = delta(1)/h(0);

            HTemp(1,0) = delta(1)/q;
            HTemp(1,1) = -1*delta(0)/q;
            HTemp(1,2) = -1*q;
            HTemp(1,3+2*i) = -1*delta(1)/q;
            HTemp(1,4+2*i) = delta(0)/q;

            //These next two terms are defined for the maximum likelihood approximation.
            //pi is minimized to select the nearest neighbor
            Eigen::MatrixXd psiTemp = HTemp*cov*HTemp.transpose()+Q;

            //Calculating the Mahalanobis distance
            double piTemp = ((zCur-hTemp)*(psiTemp.inverse())*((zCur-hTemp).transpose()))(0,0);

            //Minimum mahalanobis distance found
            //Update all values to store the data for the stored obstacle that produced the minimum distance
            if(piTemp < pi)
            {
                pi = piTemp;
                psi = psiTemp;
                h = hTemp;
                H = HTemp;
            }
        }

        //The mahalanobis distance exceeded the threshold for all existing features
        //A new feature must be created
        if(pi == newFeatureThreshold)
        {
            //Resize state and covariance matrices
            x.conservativeResize(x.rows()+2);
            cov.conservativeResize(x.rows()+2, x.cols()+2);
            cov(cov.rows()-1,cov.cols()-1) = std::numeric_limits<double>::max();

            //First add the obstacle to the state and covariance matrix
            x(newObstInd + 3) = newObstX;
            x(newObstInd + 4) = newObstX;
        }
        //TODO Determine if we need to calculate Kalman gain and update here
        //Kalman gain
        Eigen::MatrixXd K;

        //Measurement Update occurs here
        K = cov*H.transpose()*psi.inverse(); //Calculate Kalman gain
        x = x + K*(zCur-h); //Update state estimate
        Eigen::MatrixXd KH = K*H;
        cov = (Eigen::MatrixXd::Identity(KH.rows(),KH.cols()) - KH)*cov; //Update covariance matrix

    }
}

//accelerometerUpdate
//Accelerometer returns the acceleration in the x and y directions. This is used to calcalate the "observed"
//position update, which is compared to the position calculated by the motion model update
void EKFSlammer::accelerometerUpdate(const Eigen::Vector2d &previousS,
                                     const Eigen::Vector2d &gamma,
                                     const Eigen::Vector2d encoder)
{
    double timeStep = getTimeStep();
    Eigen::MatrixXd Q;

    //TODO get velocity here
    //Velocity (X&Y) of the robot determined from the enocder inputs
    Eigen::Vector2d vel;

    //Calculate the planar acceleration in the world's reference frame by transforming the vector by the inverse
    //rotation matrix and subtracting the components of gravity.,
    Eigen::Vector2d a = getRotationMatInverse()*gamma - g;

    //Calculates the position of the robot based on the previous time step and the acceleration of the robot
    Eigen::Vector2d s = previousS + vel*timeStep + 0.5*pow(timeStep,2)*a;

    //h stores the predicted position of the robot based on the motion model update
    Eigen::Vector2d h;
    h << x(0),
         x(1);

    //H is the Jacobian of the h - Jacobian of the predicted sensor measurement
    //H is a 2x5 matrix that gets mapped to a higher dimensional matrix. The computation of the
    //jacobian and mapping to the higher dimension is taking place in one step by directly assigning
    //the values to individual matrix indices.
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, cov.rows());
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
void EKFSlammer::gyroUpdate(const double &previousTheta, const double &beta)
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

void EKFSlammer::arucoUpdate(const Eigen::Vector2d &arucoMarker)
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
    H(0,0) = -delta(0)/h(0);
    H(0,1) = -delta(1)/h(0);

    H(1,0) = delta(1)/q;
    H(1,1) = -delta(0)/q;


    //Kalman gain
    Eigen::MatrixXd K;

    //Measurement Update occurs here
    K = cov*H.transpose()*(H*cov*H.transpose()+Q).inverse(); //Calculate Kalman gain
    x = x + K*(z-h); //Update state estimate
    Eigen::MatrixXd KH = K*H;
    cov = (Eigen::MatrixXd::Identity(KH.rows(),KH.cols()) - KH)*cov; //Update covariance matrix
}

void EKFSlammer::ekfCorrectionStep(const Eigen::VectorXd &kinectObstacles,
                                   const Eigen::Vector2d &previousS,
                                   const Eigen::Vector2d &gamma,
                                   const Eigen::Vector2d &encoder,
                                   const double &previousTheta,
                                   const double &beta,
                                   const Eigen::Vector2d &arucoMarker)
{
    //Update based on the obstacles detected by the Kinect. The Kinect returns the range and bearing of
    //obstacles in the environment.
    EKFSlammer::kinectUpdate(kinectObstacles);

    //Update based on the acceleration measured by the accelerometer.
    accelerometerUpdate(previousS, gamma, encoder);

    //Update based on the angular velocity measured by the gyro.
    gyroUpdate(previousTheta, beta);

    arucoUpdate(arucoMarker);

}

//TODO Handle angle updates correctly: incorporate rollover
void EKFSlammer::ekfUpdate(const control &controlIn,
                           const Eigen::VectorXd &kinectObstacles,
                           const Eigen::Vector2d &gamma,
                           const Eigen::Vector2d &encoder,
                           const double &beta,
                           const Eigen::Vector2d &arucoMarker)
{
    double deltaT;

    //Stores the previous position vector of the robot, needed for the accelerometer
    Eigen::Vector2d previousS;
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
    ekfCorrectionStep(kinectObstacles, previousS, gamma, encoder, previousTheta, beta, arucoMarker);
}
