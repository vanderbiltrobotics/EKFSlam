//
// Created by Swapnil on 4/14/2018.
//

#ifndef EKFSLAM_UTILS_H
#define EKFSLAM_UTILS_H

//Defines a control input for the motion of the robot.
//This struct matches with the input received from the ROS messages.
struct control
{
    double v; //Transational velocity of the robot
    double omega; //Angular velocity of the robot
};


#endif //EKFSLAM_UTILS_H
