#ifndef KINOVAARM_HPP
#define KINOVAARM_HPP

#include <iostream>
#include <dlfcn.h> // Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.EthCommLayerUbuntu.h>
#include <Kinova.API.EthCommandLayerUbuntu.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fstream>  // For file operations
#include <vector>   // For storing data
#include <string>   // For string operations
// #include "shared_structs.hpp"
// Define constants if not defined in KinovaTypes.h
#include <Eigen/Dense>
#include <Eigen/Core>
#ifndef COMMAND_SIZE
#define COMMAND_SIZE 7 // Adjust according to your Kinova model
#endif

//#ifndef GRAVITY_PARAM_SIZE
#define GRAVITY_PARAM_SIZE 4 // Adjust to 4 since your robot has 4 joints
//#endif

// sharedData KinovaData1;
class KinovaArm {
private:
    int result;
    int programResult = 0;
    // Handle for the library's command layer.
    void * commandLayer_handle;
    // Function pointers to the functions we need
    int(*MyInitAPI)();
    int(*MyCloseAPI)();
    int(*MyGetAngularCommand)(AngularPosition &);
    int(*MyGetAngularPosition)(AngularPosition &);
    int(*MyGetDevices)(KinovaDevice[MAX_KINOVA_DEVICE], int &);
    int(*MySetActiveDevice)(KinovaDevice);
    int(*MyGetActuatorAcceleration)(AngularAcceleration &Response);
    int(*MyGetAngularVelocity)(AngularPosition &Response);
    int(*MyMoveHome)();
    int(*MyInitFingers)();
    int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
    int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
    int(*MySetTorqueSafetyFactor)(float factor);
    int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
    int(*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);
    int(*MySetGravityVector)(float Command[3]);
    int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
    int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
    
    int(*MySetTorqueVibrationController)(float value);
    int(*MyGetAngularForceGravityFree)(AngularPosition &);
    int(*MyGetCartesianForce)(CartesianPosition &);
    int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
    int(*MyGetAngularForce)(AngularPosition & Response ); // Corrected function pointer
    int(*MyGetQuickStatus)( QuickStatus & Response); 	
    int(*MyGetAngularTorqueGravityEstimation)(float Command[GRAVITY_PARAM_SIZE]);
    int(*MySetTorqueRobotProtection)(int protectionLevel);
    int(*MySendBasicTrajectory)(TrajectoryPoint command);
    int (*MySetActuatorPID)(unsigned int address, float P, float I, float D);
    int (*MyGetSystemErrorCount)(unsigned int &);
    int (*MyGetSystemError)(unsigned int, SystemError &);
    int(*MyGetTrajectoryTorqueMode)(int &);
    int(*MyEraseAllProtectionZones)();
    int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
    int(*MySetTorqueActuatorDamping)(float Command[COMMAND_SIZE]);
    int (*MyClearErrorLog)();
    int (*MySetSwitchThreshold)(float Command[COMMAND_SIZE]);   
    int (*MySetTorqueRateLimiter)(float Command[COMMAND_SIZE]);
    int (*MySendJoystickCommand)(JoystickCommand command);
    int (*MyStartControlAPI)();
    int(*MyGetJoystickValue)(JoystickCommand &command);
    int (*MyGetGripperStatus)(Gripper &status);
    int (*MyGetControlMapping)(ControlMappingCharts &);


    std::ofstream dataFile;
    float TorqueCommand[COMMAND_SIZE];
    AngularPosition actualTorque;
    float gravityTorque[GRAVITY_PARAM_SIZE] = {0};
    AngularPosition angularPosition;
    AngularPosition angularVelocity;
    std::string filePath = "./tmp/torque_data.txt";
    long control_tick = 0;
    long motiontime = 0;
    TrajectoryPoint startAngularPosition;
    int TrajectoryTorqueMode = 0;
    int resultComm = 0;
    int resultError = 0;
    int resultFingers = 0;
    int resultControlAPI = 0;
    unsigned int ErrorCount = 0;
    SystemError errorTemp;
    JoystickCommand virtualCommand;
    

public:
    KinovaArm(double angular_pose[4]){
        setStartPose(angular_pose);
        initKinova();
    };
    ~KinovaArm(){
        cleanUpKinova();
    };

    void runTorqueControl(Eigen::VectorXd jointTorqueKinova);
    void runAngularPositionControl();
    void getAngularPosition(double angular_pose[4]);
    void getAngularVelocity(double angular_velocity[4]);
    void holdPosition();
    int getResultComm() { return resultComm; }
    void getPDTorques(Eigen::VectorXd& torqueCommand);
    int (*MySetGravityManualInputParam)(float Command[GRAVITY_PARAM_SIZE]);
    int(*MySetGravityType)(GRAVITY_TYPE Type);
    void sendGripperCommand(float joystick_state[19]);


private:
    void setStartPose(double angular_pose[4]);
    int initKinova();
    void cleanUpKinova();
    void moveRobotToStart();
    


};






#endif
