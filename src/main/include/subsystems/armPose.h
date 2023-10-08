#pragma once

#include <string>
#include <vector>


using namespace std;

class armPose{
    public:
    armPose();
    armPose(double shoulder, double elbow);
    double getShoulderPose();
    double getElbowPose();
    void setShoulderPose(double shoulder);
    void setElbowPose(double elbow);
    

    private:
    double shoulderPosition;
    double elbowPosition;


};