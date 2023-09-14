#include <string>

using namespace std;

class armPose{
    public:
    armPose();
    armPose(double shoulder, double elbow, int id, string name);
    void set(double shoulder, double elbow);
    void getNextPose();
    double getShoulderPose();
    double getElbowPose();
    int getPoseID();
    string getName();
    void setShoulderPose(double shoulder);
    void setElbowPose(double elbow);
    void setPoseID(int id);
    void setPoseName(string name);
    

    private:
    double shoulderPosition;
    double elbowPosition;
    int poseID;
    string poseName;


};