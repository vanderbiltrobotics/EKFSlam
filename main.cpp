
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

void writeToFile(double t,
                 std::ofstream &dataRobot,
                 std::ofstream &dataMap,
                 Eigen::Vector3d &actualState,
                 Eigen::VectorXd &predictedState,
                 Eigen::MatrixXd &cov,
                 Eigen::VectorXd &map)
{
    std::string output1 = std::to_string(t) + ",";
    output1+= std::to_string(actualState(0)) + ",";
    output1+= std::to_string(actualState(1)) + ",";
    output1+= std::to_string(actualState(2)) + ",";
    output1+= std::to_string(predictedState(0)) + ",";
    output1+= std::to_string(predictedState(1)) + ",";
    output1+= std::to_string(predictedState(2)) + ",";
    output1+= std::to_string(cov(0,0)) + ",";
    output1+= std::to_string(cov(1,1)) + ",";
    output1+= std::to_string(cov(2,2));

    std::string output2 = std::to_string(t) + ",";
    output2 += std::to_string((cov.rows()-3)/2) + ",";
    std::cout << cov.rows();
    for(int i = 0; i < (cov.rows()-3)/2; i++)
    {
        output2 += std::to_string(predictedState(2*i+3)) + ",";
        output2 += std::to_string(predictedState(2*i+4)) + ",";
        output2 += std::to_string(cov(2*i+3, 2*i+3)) + ",";
        output2 += std::to_string(cov(2*i+4, 2*i+4)) + ",";
    }
    output1 +="\n";
    output2 +="\n";
    dataRobot << output1;
    dataMap << output2;


}

int main() {
    double t = 0;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    Eigen::Vector3d initPos; initPos << 0, 0, M_PI/2.0;
	Robot robot(initPos);

    EKFSlammer slam(robot);

    //File for data output
    std::ofstream dataRobot ( "dataRobot.csv" );
    std::ofstream dataMap ( "dataMap.csv" );
    std::string outputRobot;
    std::string outputMap;

    outputRobot = "t, Actual X, Actual Y, Actual Theta, Estimated X, Estimated Y, Estimated Theta, ";
    outputRobot += " Covariance X, Covariance Y, Covariance Theta";

    outputMap = "t, n, Actual X, Actual Y, Estimated X, Estimated Y, Covariance X, Covariance Y";

    dataRobot << outputRobot;
    dataMap << outputMap;




    Eigen::VectorXd map = Eigen::VectorXd::Constant(6, 0.0);
    //Generating three obstacles in the obstacle area
    for(int i = 0; i < 3; i += 2)
    {
        map(i) = 3.78*((double) rand() / (RAND_MAX))-1.89; //Obstacle position in x
        map(i+1) = 2.94*((double) rand() / (RAND_MAX))+1.5; //Obstacle position in y
    }
    std::cout << map(0) << " " << map(1) << std::endl;

    Eigen::Vector3d actualState;
    Eigen::VectorXd predictedState;
    Eigen::MatrixXd cov;

    control c;
    c.v = 1;
    c.omega = 1;

    for(double i = 0; i < 3; i += 0.02) {
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
        cov = slam.getCov();
        printState(actualState, predictedState);
        t+=0.02;
        writeToFile(t, dataRobot, dataMap, actualState, predictedState, cov, map);
    }

    c.v = 0;
    c.omega = 0.4;

    for(double i = 0; i < 2; i += 0.02) {
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
        cov = slam.getCov();
        printState(actualState, predictedState);
        t+=0.02;
        writeToFile(t, dataRobot, dataMap, actualState, predictedState, cov, map);

    }

    c.v = 0.5;
    c.omega = 0.1;

    for(double i = 0; i < 2; i += 0.02) {
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

        cov = slam.getCov();

        printState(actualState, predictedState);
        t+=0.02;
        writeToFile(t, dataRobot, dataMap, actualState, predictedState, cov, map);

    }

    dataRobot.close();
    dataMap.close();


}