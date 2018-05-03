#include <iostream>
#include "EKFSlammer.h"
#include "Eigen/Dense"
#include "Utils.h"
#include "Robot.h"
#include <fstream>



int main() {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);

    Eigen::Vector3d initPos; initPos << 0,0,0;
	Robot robot(initPos);

    EKFSlammer slam;

    Eigen::VectorXd map = Eigen::VectorXd::Constant(2,0);
    map(0) = 0;
    map(1) = 5;

    control c;
    c.v = 1;
    c.omega = 1;

    robot.processControlInput(c);

    robot.stepTime(0.5);


    slam.ekfUpdate(c,
                   robot.getKinectMeasurement(map),
                   robot.getAccelerometerMeasurement(),
                   robot.getEncoderMeasurement(),
                   robot.getGyroMeasurement(),
                   robot.getArucoMeasurment());




    std::cout << "Estimated State: " << slam.getState() << std::endl;
    std::cout << "Actual State: " << robot.getActualPos();




//    control test = {0.5, 0};
//
//    std::cout << robot.getActualPos() << std::endl;
//
//    robot.processControlInput(test);
//    std::cout << "Actual Measurement : " << robot.getActualVelocity() << std::endl;
//
//    std::cout << "Encoder Measurement : " << robot.getEncoderMeasurement() << std::endl;
//
//    robot.stepTime(1);
//
//    std::cout << "Actual Position: "<< robot.getActualPos() << std::endl;

}