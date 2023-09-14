#include "subsystems/Superstructure/armPose.h"

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

void armPose::set(double shoulder, double elbow){
    setShoulderPose(shoulder);
    setElbowPose(elbow);
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