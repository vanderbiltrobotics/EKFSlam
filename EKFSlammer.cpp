//#include <d2d1_1helper.h>
#include "EKFSlammer.h"
#include <cmath>
#include <iostream>


/*TODO LIST
    todo - Determine how to add motion model and measurement noise
    todo - Determine velocity term for accelerometer update
 */


//EKFSlammer
//Initializes EKF Slammer. The robot's current pose is considered (0,0,0)
EKFSlammer::EKFSlammer(const Robot& r) : x(r.getActualPos()),
                                         cov(Eigen::Matrix3d::Constant(0)),
                                         g(Eigen::Vector2d::Constant(0)),
                                         n(0),
                                         newFeatureThreshold(1),
                                         TIME_STEP(0.02)
{
    //cov(0,0) = std::numeric_limits<double>::max();
    //cov(1,1) = std::numeric_limits<double>::max();
    //cov(2,2) = std::numeric_limits<double>::max();

}


Eigen::MatrixXd EKFSlammer::getState() const
{
    return x;
}



double EKFSlammer::getTimeStep() const
{
    return TIME_STEP; //Time Step in seconds. Constant for the purpose of the simulation
}

//getRotationMat
//Returns 2x2 rotation matrix describing the transformation from the world reference frame to the
//robot reference frame
Eigen::Matrix2d EKFSlammer::getRotationMat() const
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
Eigen::Matrix2d EKFSlammer::getRotationMatInverse() const
{
    return getRotationMat().inverse();
}


//getMotionModelUncertainty
//Returns the uncertainty in the motion model update
Eigen::Matrix3d EKFSlammer::getMotionModelUncertainty(const control &cIn) const
{
    //Odometry noise parameters
    double a1 = 0.05;   //translational velocity (TV) to estimated TV
    double a2 = 0.05;   //Rotational velocity (RV) to estimated TV
    double a3 = 0.05;   //TV to est RV
    double a4 = 0.05;   //RV to est RV
    double hnsf = 0.001; //Heading noise scale factor

    Eigen::Matrix3d error = Eigen::MatrixXd::Constant(3, 3, 0);
    // provisional, 3x3 matrix of ones
    error(0,0) = a1*pow(cIn.v,2)+a2*pow(cIn.omega,2);
    error(1,1) = a3*pow(cIn.v,2)+a4*pow(cIn.omega, 2);
    error(2,2) = sqrt(error(0,0) + error(1,1))*hnsf;
    return error;

}


//motionModelUpdate
//Calculates the predicted position (mean and covariance) of the robot based on the command passed
//const double &deltaT - Time step between updates
//const control &controlIn - Control input received from
void EKFSlammer::motionModelUpdate(const double &deltaT, const control &controlIn)
{
    //Store angular position in variable theta for readability
    double theta = x(2);

    double vOmegaRatio;
    
    // TODO: Define for small omega
    if(controlIn.omega <= 0.00001)
    {
        // Do nothing for now.
    }
    else
    {
        //Ratio of velocity to angular velocity used for motion model update
        vOmegaRatio = controlIn.v / controlIn.omega;
        //Calculate predicted mean based on motion model
        //x(0) - x position of robot
        //x(1) - y position of robot
        //x(2) - theta (angular position of robot, measured ccw from positive x)
        x(0) += -vOmegaRatio*sin(theta) + vOmegaRatio * sin(theta + controlIn.omega*deltaT);
        x(1) += vOmegaRatio*cos(theta) - vOmegaRatio * cos(theta + controlIn.omega*deltaT);
        x(2) += std::fmod(controlIn.v*deltaT, 2*M_PI);

        //Calculate Jacobian for motion model update
        //Gxt is the Jacobian for only the current pose of the robot (does not include landmark locations)
        //We do not need to calculate the update for the landmarks since the motion model does not update the
        //pose estimate for the landmarks. Therefore, there is no change in the covariance for any landmarks.

        //Note: Gxt is capital since g represents the motion model function while G represents the Jacobian for the
        //motion model function
        Eigen::Matrix3d Gxt = Eigen::Matrix3d::Identity();
        Gxt(0,2) = controlIn.v * cos(theta) / (controlIn.omega)
                   - controlIn.v * cos(theta + controlIn.omega*deltaT) / controlIn.omega;
        Gxt(1,2) = -controlIn.v * sin(theta) / controlIn.omega
                   + controlIn.v * sin(theta + controlIn.omega*deltaT) / controlIn.omega;

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
        covXX += getMotionModelUncertainty(controlIn);

        if(cov.rows() > 3) {
            //Extract the 3xM covariance matrix for xm (pose of robot with map)
            //covXM in the covariance matrix is within columns 3-N and rows 0-2 where N is number of rows/cols of cov matrix
            Eigen::MatrixXd covXM = cov.block(0, 3, 3, 3 - n);
            std::cout << Gxt.rows() << " " << Gxt.cols();
            std::cout << covXM.rows() << " " << covXM.cols();
            //Calculated updated covariance matrix for xm
            //Multiply by the Jacobian
            covXM = Gxt * covXM;
            cov.block(0, 3, 3, 3-n) = covXM;
            cov.block(3, 0, 3-n, 3) = covXM.transpose();
        }
        //Write updated covariances back to cov
        //XX Update
        cov.block<3,3>(0, 0) = covXX;

        //XM Update (3-n matrix) - n is the number of landmarks stored

        // for(int i = 0; i < 3; i++) {
        //     for(int j = 2; j < cov.cols(); j++) {
        //         cov(i, j) = covXM(i, j);
        //     }
        // }

        //MX Update (n-3 matrix) - n is the number of landmarks stored
        // for(int i = 0; i < cov.rows(); i++) {
        //     for(int j = 2; j < 3; j++) {
        //         cov(i, j) = covXM.transpose()(i, j);
        //     }
        // }
    }

}


//kinectUpdate
//Kinect returns the distance and angle (relative to the robot) of detected obstacles in the environment
//Calculates the updated position of the robot based on the measurements
void EKFSlammer::kinectUpdate(const Eigen::VectorXd &z)
{
    int obstacleIndex; //Stores the index of the obstacle that is currently being operated on.

    Eigen::Matrix2d Q;

    Q(0,0) = 0.0002502;
    Q(0,1) = -0.0000052;
    Q(1,0) = -0.0000052;
    Q(1,1) = 0.0002070;


    for(int i = 0; i < z.rows()/2; i++)
    {
        //zCur stores the current measurement being operated on
        Eigen::Vector2d zCur;
        zCur(0) = (z(2*i));
        zCur(1) = (z(2*i+1));

        //First step of the update is to use maximum likelihood approximation to determine associate the
        //measurement of the obstacle to a previously detected obstacle.

        //Maximum likelihood approximation starts by creating the hypothesis of a new landmark.
        //The location of the landmark is calculated by transforming the range-bearing measurement to the
        //world reference frame

        double newObstX = x(0) + zCur(0)*cos(zCur(1) + x(2));
        double newObstY = x(1) + zCur(0)*sin(zCur(1) + x(2));
        int newObstInd = n+1; //Index of the new obstacle is n+1



        //h stores the the predicted observation for the given landmark (range bearing).
        Eigen::Vector2d h = zCur;

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
            Eigen::Vector2d delta;
            //Add three to index to skip over (x,y,theta)
            delta << (x(2*j+3) - x(0)),
                    (x(2*j+1+3) - x(1));

            //Calculating distance between expected position of landmark and expected position of robot (r^2)
            double q = delta.dot(delta);

            //h stores the the predicted observation for the given landmark (range bearing).
            Eigen::Vector2d hTemp;
            hTemp << sqrt(q),
                    (atan2(delta(1), delta(0)) - x(2));

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
            HTemp(1,2) = -1;
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
            cov.conservativeResize(cov.rows()+2, cov.cols()+2);
            //cov(cov.rows()-1,cov.cols()-1) = std::numeric_limits<double>::max();
            //First add the obstacle to the state and covariance matrix
            x(newObstInd + 2) = newObstX;
            x(newObstInd + 3) = newObstY;

            Eigen::Vector2d delta;
            //Add three to index to skip over (x,y,theta)
            delta << (x(newObstInd + 2) - x(0)),
                    (x(newObstInd + 3) - x(1));

            //Calculating distance between expected position of landmark and expected position of robot (r^2)
            double q = delta.dot(delta);

            H.conservativeResize(2, cov.cols());
            //Building Jacobian matrix H with new obstacle included
            H(0,0) = -1*delta(0)/h(0);
            H(0,1) = -1*delta(1)/h(0);
            //H(0,2) = 0; In here just for readability
            H(0,newObstInd + 2) = delta(0)/h(0);
            H(0,newObstInd + 3) = delta(1)/h(0);

            H(1,0) = delta(1)/q;
            H(1,1) = -1*delta(0)/q;
            H(1,2) = -1*q;
            H(1,newObstInd + 2) = -1*delta(1)/q;
            H(1,newObstInd + 3) = delta(0)/q; //TESTING CONTINUES HERE. VERIFY THAT H IS BEING CALCULATED CORRECTLY.
            //TODO, make sure indices for building H normally are correct

        }

        //TODO Determine if we need to calculate Kalman gain and update here
        //Kalman gain
        Eigen::MatrixXd K;

        //Measurement Update occurs here
        psi = H*cov*H.transpose()+Q;
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

    //Calculates the orientation of the robot based on the angular velocity and previous orientation
    double theta = previousTheta + beta*timeStep;

    //h stores the predicted orientation of the robot based on the motion model update
    double h = x(2);

    //H is the Jacobian of the h - Jacobian of the predicted sensor measurement
    //H is a 1x5 matrix that gets mapped to a higher dimensional matrix. The computation of the
    //jacobian and mapping to the higher dimension is taking place in one step by directly assigning
    //the values to individual matrix indices.
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1,cov.rows());
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(1,1);

    Q(0,0) = 0.000174; //TODO INSERT GYRO STANDARD DEVIATION HERE
    H(0,2) = 1;

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

    Eigen::Matrix2d Q; 
    Q(0,0) = 0.002687;
    Q(1,0) = 0.0;
    Q(0,1) = 0.0;
    Q(1,1) = 0.002687;
    // Q(0,0) = 0.000007185;
    // Q(1,0) = -0.000000057;
    // Q(0,1) = -0.000000057;
    // Q(1,1) = 0.000007047;

    //delta stores the difference in expected position of marker (0,0) and expected position of robot
    Eigen::Vector2d delta;
    delta << (0.0 - x(0)),
             (0.0 - x(1));

    double q = delta.dot(delta);

    //h stores the the predicted observation for the given landmark.
    Eigen::Vector2d h;
    h << sqrt(q), fmod((atan2(delta(1),delta(0)) - M_PI/2.0 + x(2)), 2*M_PI);
    std::cout << "Dat fat h(1) val: " << h(1) << std::endl;
    std::cout << delta << std::endl;

    //Location of the AruCo marker, which is known to be (0,0)
    //Eigen::Vector2d z;
    //z << 0.0, 0.0;

    //H is the Jacobian of the h - Jacobian of the predicted sensor measurement
    //H is a 2x5 matrix that gets mapped to a higher dimensional matrix. The computation of the
    //jacobian and mapping to the higher dimension is taking place in one step by directly assigning
    //the values to individual matrix indices.
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,cov.rows());
    H(0,0) = -delta(0)/h(0);
    H(0,1) = -delta(1)/h(0);

    H(1,0) = delta(1)/q;
    H(1,1) = -delta(0)/q;

    H(1,2) = -1;


    //Kalman gain
    Eigen::MatrixXd K;

    //Measurement Update occurs here
    K = cov*H.transpose()*(H*cov*H.transpose()+Q).inverse(); //Calculate Kalman gain
    x = x + K*(arucoMarker-h); //Update state estimate
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
    //accelerometerUpdate(previousS, gamma, encoder);

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
//    std::cout << "Control: " << controlIn.v << " " << controlIn.omega << std::endl;
//    std::cout << "Kinect: " << kinectObstacles << std::endl;
//    std::cout << "accelerometer: " <<  gamma << std::endl;
//    std::cout << "encoder: " <<  encoder << std::endl;
//    std::cout << "gyro: " <<  beta << std::endl;
//    std::cout << "aruco: " <<  arucoMarker << std::endl;

    //Stores the previous position vector of the robot, needed for the accelerometer
    Eigen::Vector2d previousS;
    previousS << x(0),x(1);

    //Stores the previous position angle of the robot, needed for the gyro
    double previousTheta = x(2);

    //First step of the EKF update for SLAM
    //This is predicting the updated position and covariance based on only the motion model of the robot
    motionModelUpdate(TIME_STEP, controlIn); //Calculates the estimated new position based on the motion model.




    //Second step of EKF SLAM.
    //This is updating the prediction based on the data from the sensors. The observed values from the sensors
    //are compared to the predicted value from the previous step. Depending on error of the motion model
    //and the error of the sensor model, the update is weighted towards one or the other.
    ekfCorrectionStep(kinectObstacles, previousS, gamma, encoder, previousTheta, beta, arucoMarker);


}
