#ifndef VREP_H
#define VREP_H

#include "Robot.h"
#include "Vrep.h"
#include <vector>

enum Error {NO_ERROR, ARGUMENT_ERROR, CONNECTION_ERROR_1, CONNECTION_ERROR_2};
#define UISPEEDHANDLE 3
#define UITURNHANDLE 4
#define UILOCBUTTON 5
#define UIMAPBUTTON 6
#define UILOCMAPBUTTON 7

class Vrep
{
public:
    //FUNCTIONS
    Vrep(int argc,char* argv[]);
    ~Vrep();
    void loop(Robot* robot);

    //PROPERTIES
    int error;

private:
    //PROPERTIES
    int portNb;
    int clientID;
    int robotHandle;
    int leftMotorHandle;
    int rightMotorHandle;
    int sensorHandle;
    int uiHandle;
    int uiSpeedHandle;
    int uiTurnHandle;
    std::vector<int> mapHandle;
    //std::vector<int> robotMapHandle;
    simxFloat lastRotationL;
    simxFloat lastRotationR;
    simxUChar red[12];
    simxUChar purple[12];
};

#endif // VREP_H
