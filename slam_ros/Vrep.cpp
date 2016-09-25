#include "Vrep.h"
#include <stdio.h>
#include <stdlib.h>


//ctor
Vrep::Vrep(int argc,char* argv[])
{
    portNb = 0;
    error = 0;

    if (argc>=9)
    {
        portNb=atoi(argv[1]);
        robotHandle = atoi(argv[2]);
        leftMotorHandle=atoi(argv[3]);
        rightMotorHandle=atoi(argv[4]);
        sensorHandle=atoi(argv[5]);
        uiHandle = atoi(argv[6]);
        uiSpeedHandle = atoi(argv[7]);
        uiTurnHandle = atoi(argv[8]);
        mapHandle.reserve(1000);
        mapHandle.push_back(0);
        //robotMapHandle.reserve(100);
        //robotMapHandle.push_back(0);
        red[0] = 255;
        red[1] = 0;
        red[2] = 100;
        red[3] = 0;
        red[4] = 0;
        red[5] = 0;
        red[6] = 0, red[7] = 0;
        red[8] = 0;
        red[9] = 0;
        red[10] = 0;
        red[11] = 0;
        purple[0] = 160;
        purple[1] = 32;
        purple[2] = 240;
        purple[3] = 0;
        purple[4] = 0;
        purple[5] = 0;
        purple[6] = 0, purple[7] = 0;
        purple[8] = 0;
        purple[9] = 0;
        purple[10] = 0;
        purple[11] = 0;
    }
    else
    {
        error = ARGUMENT_ERROR;
        printf("Indicate following arguments: 'portNumber leftMotorHandle rightMotorHandle sensorHandle uiHandle uiSpeedHandle uiTurnHandle'!\n");
        extApi_sleepMs(5000);
        delete this;
    }

    int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
    if (clientID ==-1)
    {
        error = CONNECTION_ERROR_1;
        printf("possible connection Timeout...");
        delete this;
    }

}

//dtor
Vrep::~Vrep()
{
    if(error == NO_ERROR)
    {
        simxFinish(clientID);
    }
}

void Vrep::loop(Robot* robot)
{
    extApi_sleepMs(500);
    if(simxGetConnectionId(clientID) != -1)
    {
        int uiSpeedSlider = 500;
        int uiTurnSlider = 500;
        int uiLocButton = 0;
        int uiMapButton = 0;
        int uiLocMapButton = 0;
        char trigger = 0;
        simxUChar detection = 0;
        simxFloat detect[3] = {0, 0, 0};
        float motorSpeeds[2];
        simxFloat* vreppos = new simxFloat(3);
        simxFloat* vrepori = new simxFloat(3);

        simxGetUISlider(clientID, uiHandle, UISPEEDHANDLE, &uiSpeedSlider, simx_opmode_streaming);
        simxGetUISlider(clientID, uiHandle, UITURNHANDLE, &uiTurnSlider, simx_opmode_streaming);
        simxGetUIButtonProperty(clientID, uiHandle, UILOCBUTTON, &uiLocButton, simx_opmode_streaming);
        simxGetUIButtonProperty(clientID, uiHandle, UIMAPBUTTON, &uiMapButton, simx_opmode_streaming);
        simxGetUIButtonProperty(clientID, uiHandle, UILOCMAPBUTTON, &uiLocMapButton, simx_opmode_streaming);
        simxReadProximitySensor(clientID, sensorHandle, &detection, detect, 0, 0, simx_opmode_streaming);
        // simxGetObjectPosition(clientID, robotHandle, -1, vreppos, simx_opmode_streaming);
        while (simxGetConnectionId(clientID)!=-1)
        {
            simxGetUISlider(clientID, uiHandle, UISPEEDHANDLE, &uiSpeedSlider, simx_opmode_buffer);
            simxGetUISlider(clientID, uiHandle, UITURNHANDLE, &uiTurnSlider, simx_opmode_buffer);
            simxGetUIButtonProperty(clientID, uiHandle, UILOCBUTTON, &uiLocButton, simx_opmode_buffer);
            simxGetUIButtonProperty(clientID, uiHandle, UIMAPBUTTON, &uiMapButton, simx_opmode_buffer);
            simxGetUIButtonProperty(clientID, uiHandle, UILOCMAPBUTTON, &uiLocMapButton, simx_opmode_buffer);
            simxReadProximitySensor(clientID, sensorHandle, &detection, detect, 0, 0, simx_opmode_buffer);
            //std::cout << " " << detect[2] << " ";

            //Modify motors according to the desired turn ratio
            if(uiTurnSlider > 900)
            {
                motorSpeeds[0] = 1;
                motorSpeeds[1] = -1;
            }
            else if(uiTurnSlider < 100)
            {
                motorSpeeds[0] = -1;
                motorSpeeds[1] = 1;
            }
            else if(uiTurnSlider < 350 || uiTurnSlider > 650)
            {
                motorSpeeds[0] = (uiTurnSlider-500)/800.0+0.5;
                motorSpeeds[1] = (500 - uiTurnSlider)/800.0+0.5;
                //std::cout << "left: " << motorSpeeds[0] << "right: " << motorSpeeds[1];
            }
            else
            {
                //robot goes straight within a certain treshold
                motorSpeeds[0] = 5.0;
                motorSpeeds[1] = 5.0;
            }

            //Modify motors according to overall speed
            if(uiSpeedSlider < 200)
            {
                //robot does not move within a certain treshold
                motorSpeeds[0] = 0;
                motorSpeeds[1] = 0;
            }
            else
            {
                motorSpeeds[0]*= uiSpeedSlider*0.0002*3.1415;
                motorSpeeds[1]*= uiSpeedSlider*0.0002*3.1415;
            }

            if(((uiLocButton & sim_buttonproperty_isdown) == 0) && ((uiMapButton & sim_buttonproperty_isdown) == 0))
            {
                trigger = 0;
            }
            if(trigger == 0)
            {
                if((uiLocButton & sim_buttonproperty_isdown) != 0)
                {
                    simxFloat left;
                    simxFloat right;
                    simxGetJointPosition(clientID, leftMotorHandle, &left, simx_opmode_blocking);
                    simxGetJointPosition(clientID, rightMotorHandle, &right, simx_opmode_blocking);
                    double rot[2];
                    rot[0] = left - lastRotationL;
                    rot[1] = right - lastRotationR;
                    std::vector<line> lines;
                    lines.clear();
                    robot->localize(rot, lines);
                    lastRotationL = left;
                    lastRotationR = right;

                    simxGetObjectPosition(clientID, robotHandle, -1, vreppos, simx_opmode_blocking);
                    simxGetObjectOrientation(clientID, robotHandle, -1, vrepori, simx_opmode_blocking);
                    std::cout<< " VREP: pos: " << vreppos[0] << "  " << vreppos[1] << "\t ORIENTATION: " << vrepori[0] << " " << vrepori[1] << " " << vrepori[2];
                    std::cout<< "\n ROBOT: pos: " << robot->xPos << "  " << robot->yPos << "  \t ORIENTATION:" << robot->thetaPos;
                    std::cout <<  "\n ";

                    /*simxFloat pos[3];
                    simxCreateDummy(clientID, (robot->P_t0[0] + robot->P_t0[4]), purple,  &robotMapHandle.back(), simx_opmode_blocking);
                    pos[0] = robot->xPos;
                    pos[1] = robot->yPos;
                    pos[2] = 0.6;
                    simxSetObjectPosition(clientID, robotMapHandle.back(), -1, pos, simx_opmode_oneshot);*/

                    uiLocButton ^= sim_buttonproperty_isdown;
                    simxSetUIButtonProperty(clientID, uiHandle, UILOCBUTTON, uiLocButton, simx_opmode_oneshot);
                    trigger = 1;
                }
                if((uiMapButton & sim_buttonproperty_isdown) != 0)
                {
                    if(detection)
                    {
                        simxFloat pos[3];
                        simxCreateDummy(clientID, 0.1, red,  &mapHandle.back(), simx_opmode_blocking);
                        robot->robot2World1(detect[2], pos);
                        pos[2] = 0.1;
                        simxSetObjectPosition(clientID, mapHandle.back(), -1, pos, simx_opmode_oneshot);
                    }
                    uiMapButton ^= sim_buttonproperty_isdown;
                    simxSetUIButtonProperty(clientID, uiHandle, UIMAPBUTTON, uiMapButton, simx_opmode_oneshot);
                    trigger = 1;
                }
                if((uiLocMapButton & sim_buttonproperty_isdown) != 0)
                {
                    uiLocMapButton ^= sim_buttonproperty_isdown;
                    simxSetUIButtonProperty(clientID, uiHandle, UILOCMAPBUTTON, uiLocMapButton, simx_opmode_oneshot);
                    trigger = 1;

                    if(detection)
                    {
                        simxFloat simPos[3];
                        simxGetObjectPosition(clientID, mapHandle.back(), -1, simPos, simx_opmode_blocking);
                        std::cout <<  "\n SENDING" << simPos[0] << "  " << simPos[1] << "\n";
                        double pos[3];
                        pos[0] = simPos[0]; pos[1] = simPos[1];
                        std::cout <<  "\n SENDING" << pos[0] << "  " << pos[1] << "\n";

                        simxFloat left;
                        simxFloat right;
                        simxGetJointPosition(clientID, leftMotorHandle, &left, simx_opmode_blocking);
                        simxGetJointPosition(clientID, rightMotorHandle, &right, simx_opmode_blocking);
                        double rot[2];
                        rot[0] = left - lastRotationL;
                        rot[1] = right - lastRotationR;
                        std::vector<line> lines;
                        lines.clear();
                        robot->localize(rot, lines);
                        lastRotationL = left;
                        lastRotationR = right;

                        simxGetObjectPosition(clientID, robotHandle, -1, vreppos, simx_opmode_blocking);
                        simxGetObjectOrientation(clientID, robotHandle, -1, vrepori, simx_opmode_blocking);
                        std::cout<< " VREP: pos: " << vreppos[0] << "  " << vreppos[1] << "  " << vreppos[2] << "\t ORIENTATION: " << vrepori[0] << " " << vrepori[1] << " " << vrepori[2];
                        std::cout<< "\n ROBOT: pos: " << robot->xPos << "  " << robot->yPos << "  \t ORIENTATION:" << robot->thetaPos;
                        std::cout <<  "\n";
                    }
                    else
                    {
                        std::cout << "\n NO DETECTIONS! \n COMMAND IGNORED \n";
                    }

                }

            }
            //std::cout << robot->xPos << " " << robot->yPos << " " << lastRotationL << " " << lastRotationR << "\n";
            //std::cout << robot->thetaPos << "\n";
            simxSetJointTargetVelocity(clientID,leftMotorHandle,motorSpeeds[0],simx_opmode_oneshot);
            simxSetJointTargetVelocity(clientID,rightMotorHandle,motorSpeeds[1],simx_opmode_oneshot);
            simxSetFloatSignal(clientID, "varianceXAxis", robot->P_t0[0], simx_opmode_oneshot);
            simxSetFloatSignal(clientID, "varianceYAxis", robot->P_t0[4], simx_opmode_oneshot);
            simxSetFloatSignal(clientID, "varianceX", robot->xPos, simx_opmode_oneshot);
            simxSetFloatSignal(clientID, "varianceY", robot->yPos, simx_opmode_oneshot);
            //simxSetFloatSignal(clientID, "varianceTheta", robot->thetaPos, simx_opmode_oneshot);
            extApi_sleepMs(5);
        }
    }
    else
    {
        error = CONNECTION_ERROR_2;
    }
}

/*if (simxReadProximitySensor(clientID,sensorHandle,&sensorTrigger,NULL,NULL,NULL,simx_opmode_streaming)==simx_return_ok)
			{ // We succeeded at reading the proximity sensor
				int simulationTime=simxGetLastCmdTime(clientID);
				if (simulationTime-driveBackStartTime<3000)
				{ // driving backwards while slightly turning:
					motorSpeeds[0]=-3.1415f*5.0f;
					motorSpeeds[1]=-3.1415f*2.5f;
				}
				else
				{ // going forward:
					motorSpeeds[0]=100*3.1415f;
					motorSpeeds[1]=100*3.1415f;
					if (sensorTrigger)
						driveBackStartTime=simulationTime; // We detected something, and start the backward mode
				}

			}*/
/*if (simxReadProximitySensor(clientID,sensorHandle,&sensorTrigger,NULL,NULL,NULL,simx_opmode_blocking)==simx_return_ok){
	}*/
