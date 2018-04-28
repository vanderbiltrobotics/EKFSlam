//
// Created by Swapnil on 4/27/2018.
//

#ifndef EKFSLAM_ROBOT_H
#define EKFSLAM_ROBOT_H


#include "Eigen/Dense"
#include "Utils.h"

#include <iostream>
#include <chrono>
#include <random>

class Robot {
private:
    Eigen::Vector3d actualPos; // (x,y,theta)
    Eigen::Vector2d actualVelocity;  // (vx, wx)
    Eigen::Vector2d actualAcceleration; //(ax,ay)

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    std::normal_distribution<double> vDistribution;
    std::normal_distribution<double> omegaDistribution;

    std::normal_distribution<double> kinectRDistribution;
    std::normal_distribution<double> kinectThetaDistribution;

    std::normal_distribution<double> arucoXDistribution;
    std::normal_distribution<double> arucoYDistribution;

    std::normal_distribution<double> encoderVDistribution;
    std::normal_distribution<double> encoderOmegaDistribution;

    std::normal_distribution<double> gyroDistribution;

    std::normal_distribution<double> accelerometerXDistribution;
    std::normal_distribution<double> accelerometerYDistribution;


public:
    Robot(Eigen::Vector3d initPos): actualPos(initPos),
                                    actualVelocity(Eigen::Vector2d::Constant(0)),
                                    actualAcceleration(Eigen::Vector2d::Constant(0)),
                                    vDistribution(0,0.05),
                                    omegaDistribution(0,0.05),
                                    kinectRDistribution(0, 1),
                                    kinectThetaDistribution(0, 1),
                                    arucoXDistribution(0, 0.002687),
                                    arucoYDistribution(0, 0.002687),
                                    encoderVDistribution(0, 0.05),
                                    encoderOmegaDistribution(0, 0.05),
                                    gyroDistribution(0, 1),
                                    accelerometerXDistribution(0, 1),
                                    accelerometerYDistribution(0, 1)


    {
    }

    Eigen::Vector3d getActualPos()
    {
        return actualPos;
    }

    Eigen::Vector2d getActualVelocity()
    {
        return actualVelocity;
    }

    Eigen::Vector3d getAcceleration()
    {
        return actualPos;
    }

    void processControlInput(control in)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);

        actualVelocity(0) = in.v + vDistribution(generator);
        actualVelocity(1) = in.omega + omegaDistribution(generator);
    }

    void stepTime(double deltaT)
    {
        //Store angular position in variable theta for readability
        double theta = actualPos(2);

        //Ratio of velocity to angular velocity used for motion model update
        double vOmegaRatio = actualVelocity(0)/actualVelocity(1);

        //Calculate actual position of the robot based on the actual velocities
        actualPos(0) += -vOmegaRatio*sin(theta) + vOmegaRatio*sin(theta + actualVelocity(1)*deltaT);
        actualPos(1) += vOmegaRatio*cos(theta) - vOmegaRatio*cos(theta + actualVelocity(1)*deltaT);
        actualPos(2) += actualVelocity(1)*deltaT;
    }

    double getGyroMeasurement()
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);

        return actualVelocity(2) + gyroDistribution(generator);
    }

    Eigen::Vector2d getAccelerometerMeasurement()
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);

        Eigen::Vector2d errors;
        errors << accelerometerXDistribution(generator),
                    accelerometerYDistribution(generator);

        return actualAcceleration + errors;
    }


    Eigen::Vector2d getArucoMeasurment()
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);

        Eigen::Vector2d errors;
        errors << arucoXDistribution(generator),
                arucoYDistribution(generator);

        return errors;
    }


    Eigen::Vector2d getKinectMeasurement(Eigen::VectorXd obstacles)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);

        Eigen::VectorXd estLocs = Eigen::VectorXd::Constant(obstacles.rows(), 1, 0);
        for(int i = 0; i < obstacles.rows();i+=2)
        {
            Eigen::VectorXd delta;
            //Add three to index to skip over (x,y,theta)
            delta << (obstacles(i) - actualPos(0)),
                    (obstacles(i+1) - actualPos(1));

            //Calculating distance between expected position of landmark and expected position of robot (r^2)
            double q = delta.dot(delta);

            //h stores the the predicted observation for the given landmark (range bearing).
            estLocs(i) = sqrt(q) + kinectRDistribution(generator);
            estLocs(i+1) = atan2(delta(1),delta(0)) - actualPos(2) + kinectRDistribution(generator);
        }

        return estLocs;
    }


    Eigen::Vector2d getEncoderMeasurement()
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);

        Eigen::Vector2d errors;
        errors << encoderVDistribution(generator),
                encoderOmegaDistribution(generator);

        return actualVelocity + errors;
    }




};



#endif //EKFSLAM_ROBOT_H
