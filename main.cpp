#include <iostream>
#include "EKFSlammer.h"
#include "Eigen/Dense"
#include "Utils.h"
#include "Robot.h"



int main() {
    Eigen::Vector3d initPos; initPos << 0,0,0;
	Robot robot(initPos);

    control test = {0.5, 0};

    std::cout << robot.getActualPos() << std::endl;

    robot.processControlInput(test);
    std::cout << "Actual Measurement : " << robot.getActualVelocity() << std::endl;

    std::cout << "Encoder Measurement : " << robot.getEncoderMeasurement() << std::endl;

    robot.stepTime(1);

    std::cout << "Actual Position: "<< robot.getActualPos() << std::endl;

}