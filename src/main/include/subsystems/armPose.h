#include <string>
#include <vector>


using namespace std;

class armPose{
    public:
    armPose();
    armPose(double shoulder, double elbow, int id, string name);
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