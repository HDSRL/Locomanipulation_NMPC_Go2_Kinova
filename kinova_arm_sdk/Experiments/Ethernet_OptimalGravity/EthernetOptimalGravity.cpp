#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.EthCommandLayerUbuntu.h>
#include <unistd.h>
//Note that under windows, you may/will have to perform other #include
using namespace std;
int main()
{
        int result;
        int programResult = 0;
        //Handle for the library's command layer.
        void * commandLayer_handle;
        //Function pointers to the functions we need
        int(*MyInitAPI)();
        int(*MyCloseAPI)();
        int(*MyGetAngularCommand)(AngularPosition &);
        int(*MyGetAngularPosition)(AngularPosition &);
        int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
        int(*MySetActiveDevice)(KinovaDevice device);
        int(*MyGetActuatorAcceleration)(AngularAcceleration &Response);
        int(*MyGetAngularVelocity)(AngularPosition &Response);
        int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
        int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
        int(*MySetTorqueSafetyFactor)(float factor);
        int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
        int(*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);
        int(*MySetGravityVector)(float Command[3]);
        int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
        int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
        int(*MySetGravityType)(GRAVITY_TYPE Type);
        int(*MyGetAngularForceGravityFree)(AngularPosition &);
        int(*MyGetCartesianForce)(CartesianPosition &);
        //We load the library (Under Windows, use the function LoadLibrary)
        commandLayer_handle = dlopen("Kinova.API.EthCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "Ethernet_InitAPI");
        MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "Ethernet_CloseAPI");
        MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularCommand");
        MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularPosition");
        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "Ethernet_GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "Ethernet_SetActiveDevice");
        MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "Ethernet_GetActuatorAcceleration");
        MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularVelocity");
        MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "Ethernet_RunGravityZEstimationSequence");
        MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "Ethernet_SwitchTrajectoryTorque");
        MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "Ethernet_SetTorqueSafetyFactor");
        MySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "Ethernet_SendAngularTorqueCommand");
        MySendCartesianForceCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "Ethernet_SendCartesianForceCommand");
        MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "Ethernet_SetGravityVector");
        MySetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetGravityPayload");
        MySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetGravityOptimalZParam");
        MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "Ethernet_SetGravityType");
        MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularForceGravityFree");
        MyGetCartesianForce = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "Ethernet_GetCartesianForce");
        //Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) ||
                (MySwitchTrajectoryTorque == NULL))
        {
                cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
                programResult = 0;
        }
        else
        {
                cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
                result = (*MyInitAPI)();
                int resultComm;
                AngularPosition DataCommand;
                // Get the angular command to test the communication with the robot
                resultComm = MyGetAngularCommand(DataCommand);
                cout << "Initialization's result :" << result << endl;
                cout << "Communication result :" << resultComm << endl;
                // If the API is initialized and the communication with the robot is working
                if (result == 1 && resultComm == 1)
                {
                        // Choose robot type
                        ROBOT_TYPE type = MICO_6DOF_SERVICE;
                        double OptimalzParam[OPTIMAL_Z_PARAM_SIZE];
                        // Run identification sequence
                        // CAUTION READ THE FUNCTION DOCUMENTATION BEFORE
                        MyRunGravityZEstimationSequence(type, OptimalzParam);
                }
                cout << endl << "C L O S I N G   A P I" << endl;
                result = (*MyCloseAPI)();
                programResult = 1;
        }
        dlclose(commandLayer_handle);
        return programResult;
}