#include "subsystems/armPose.h"

armPose::armPose(double shoulder, double elbow, int id, string name)
  : shoulderPosition(shoulder),
    elbowPosition(elbow),
    poseID(id),
    poseName(name)
{

}

armPose::armPose(){
    shoulderPosition = 0;
    elbowPosition = 0;
    poseID = 0;
    poseName = "";
}

double armPose::getShoulderPose(){
    return shoulderPosition;
}

double armPose::getElbowPose(){
    return elbowPosition;
}

int armPose::getPoseID(){
    return poseID;
}

string armPose::getName(){
    return poseName;
}

void armPose::setShoulderPose(double shoulder){
    shoulderPosition = shoulder;
}

void armPose::setElbowPose(double elbow){
    elbowPosition = elbow;
}

void armPose::setPoseID(int id){
    poseID = id;
}

void armPose::setPoseName(string name){
    poseName = name;
}

// int armPose::getNextPose(string name){
//     if(name.compare("ground cone")==0){

//     }
// }