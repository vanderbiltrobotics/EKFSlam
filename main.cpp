
#include "EKFSlammer.h"
#include "Eigen/Dense"
#include "Utils.h"
#include "Robot.h"
#include <fstream>
#include <iostream>
#include <cmath>


void printState(Eigen::Vector3d &actual, Eigen::VectorXd &predicted)
{
    std::cout << actual(0) <<  " " << predicted(0) << std::endl;
    std::cout << actual(1) <<  " " << predicted(1) << std::endl;
    std::cout << actual(2) <<  " " << predicted(2) << std::endl;
    std::cout << "\n";

}

int main() {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    Eigen::Vector3d initPos; initPos << 0, 0, M_PI/2.0;
	Robot robot(initPos);

    EKFSlammer slam(robot);



    Eigen::VectorXd map = Eigen::VectorXd::Constant(4, 0.0);
    //Generating three obstacles in the obstacle area
    for(int i = 0; i < 4; i += 2)
    {
        map(i) = 3.78*((double) rand() / (RAND_MAX))-1.89; //Obstacle position in x
        map(i+1) = 2.94*((double) rand() / (RAND_MAX))+1.5; //Obstacle position in y
    }
    std::cout << map(0) << " " << map(1) << std::endl;

    Eigen::Vector3d actualState;
    Eigen::VectorXd predictedState;

    control c;
    c.v = 1;
    c.omega = 1;

    for(double i = 0; i < 7; i += 0.02) {
        robot.input(c);
        std::cout << "TIME STEP: "  << i << std::endl;
        robot.stepTime(0.02);


        slam.ekfUpdate(c,
                       robot.getKinectMeasurement(map),
                       robot.getAccelerometerMeasurement(),
                       robot.getEncoderMeasurement(),
                       robot.getGyroMeasurement(),
                       robot.getArucoMeasurment());
        actualState = robot.getActualPos();
        predictedState = slam.getState();
        printState(actualState, predictedState);
    }

    c.v = 0;
    c.omega = 0.4;

    for(int i = 0; i < 5; i += 0.02) {
        robot.input(c);

        robot.stepTime(0.02);


        slam.ekfUpdate(c,
                       robot.getKinectMeasurement(map),
                       robot.getAccelerometerMeasurement(),
                       robot.getEncoderMeasurement(),
                       robot.getGyroMeasurement(),
                       robot.getArucoMeasurment());
        actualState = robot.getActualPos();
        predictedState = slam.getState();
        printState(actualState, predictedState);
    }

    c.v = 0.5;
    c.omega = 0.1;

    for(int i = 0; i < 5; i += 0.02) {
        robot.input(c);

        robot.stepTime(0.02);

        slam.ekfUpdate(c,
                       robot.getKinectMeasurement(map),
                       robot.getAccelerometerMeasurement(),
                       robot.getEncoderMeasurement(),
                       robot.getGyroMeasurement(),
                       robot.getArucoMeasurment());

        actualState = robot.getActualPos();

        predictedState = slam.getState();

        printState(actualState, predictedState);
    }
}