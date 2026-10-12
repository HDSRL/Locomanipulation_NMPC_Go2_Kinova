#include "KinovaArm.hpp"

int KinovaArm::initKinova(){
    // We load the library (Under Windows, use the function LoadLibrary)
    commandLayer_handle = dlopen("Kinova.API.EthCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
    if (!commandLayer_handle) {
        std::cerr << "Failed to load Kinova.API.EthCommandLayerUbuntu.so: " << dlerror() << std::endl;
        return -1;
    }

    // Clear any existing errors
    dlerror();

    // Load all required functions
    MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "Ethernet_InitAPI");
    MyInitFingers = (int(*)()) dlsym(commandLayer_handle, "Ethernet_InitFingers"); // Corrected function loading
    MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "Ethernet_CloseAPI");
    MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularCommand");
    MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularPosition");
    MyGetDevices = (int(*)(KinovaDevice[MAX_KINOVA_DEVICE], int &)) dlsym(commandLayer_handle, "Ethernet_GetDevices");
    MySetActiveDevice = (int(*)(KinovaDevice)) dlsym(commandLayer_handle, "Ethernet_SetActiveDevice");
    MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "Ethernet_GetActuatorAcceleration");
    MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularVelocity");
    MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "Ethernet_RunGravityZEstimationSequence");
    MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "Ethernet_SwitchTrajectoryTorque");
    MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "Ethernet_SetTorqueSafetyFactor");
    MySendAngularTorqueCommand = (int(*)(float[COMMAND_SIZE])) dlsym(commandLayer_handle, "Ethernet_SendAngularTorqueCommand");
    MySendCartesianForceCommand = (int(*)(float[COMMAND_SIZE])) dlsym(commandLayer_handle, "Ethernet_SendCartesianForceCommand");
    MySetGravityVector = (int(*)(float[3])) dlsym(commandLayer_handle, "Ethernet_SetGravityVector");
    MySetGravityPayload = (int(*)(float[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetGravityPayload");
    MySetGravityOptimalZParam = (int(*)(float[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetGravityOptimalZParam");
    MySetGravityType = (int(*)(GRAVITY_TYPE)) dlsym(commandLayer_handle, "Ethernet_SetGravityType");
    MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularForceGravityFree");
    MyGetCartesianForce = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "Ethernet_GetCartesianForce");
    MySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "Ethernet_SetTorqueVibrationController");
    MySetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "Ethernet_SetTorqueControlType");
    MyGetAngularForce = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularForce"); // Corrected loading
    MyGetAngularTorqueGravityEstimation = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "Ethernet_GetAngularTorqueGravityEstimation");
    MyGetQuickStatus = (int(*)(QuickStatus &)) dlsym(commandLayer_handle, "Ethernet_GetQuickStatus"); 	
    MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "Ethernet_MoveHome");
    MySetTorqueRobotProtection = (int(*)(int)) dlsym(commandLayer_handle, "Ethernet_SetTorqueRobotProtection");
    int (*MySetEthernetConfiguration)(EthernetConfiguration* config) = (int (*)(EthernetConfiguration * config)) dlsym(commandLayer_handle, "Ethernet_SetEthernetConfiguration");
    int (*MyGetEthernetConfiguration)(EthernetConfiguration* config) = (int (*)(EthernetConfiguration * config)) dlsym(commandLayer_handle, "Ethernet_GetEthernetConfiguration");
    int (*MyEthernet_InitEthernetAPI)(EthernetCommConfig* config) = (int (*)(EthernetCommConfig * config)) dlsym(commandLayer_handle, "Ethernet_InitEthernetAPI");
    MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"Ethernet_SendBasicTrajectory");
    MySetActuatorPID = (int (*)(unsigned int, float, float, float )) dlsym(commandLayer_handle,"Ethernet_SetActuatorPID");
    MyGetSystemErrorCount = (int (*)(unsigned int &)) dlsym(commandLayer_handle,"Ethernet_GetSystemErrorCount");
    MyGetSystemError = (int (*)(unsigned int, SystemError &)) dlsym(commandLayer_handle,"Ethernet_GetSystemError");
    MyGetTrajectoryTorqueMode = (int(*)(int &)) dlsym(commandLayer_handle, "Ethernet_GetTrajectoryTorqueMode");
    MyEraseAllProtectionZones = (int(*)()) dlsym(commandLayer_handle, "Ethernet_EraseAllProtectionZones");
    MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"Ethernet_SendAdvanceTrajectory");
    MySetTorqueActuatorDamping = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetTorqueActuatorDamping");
    MyClearErrorLog = (int (*)()) dlsym(commandLayer_handle,"Ethernet_ClearErrorLog");
    MyGetSystemError = (int (*)(unsigned int, SystemError &)) dlsym(commandLayer_handle,"Ethernet_GetSystemError");
    MyGetTrajectoryTorqueMode = (int(*)(int &)) dlsym(commandLayer_handle, "Ethernet_GetTrajectoryTorqueMode");
    MySetGravityManualInputParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetGravityManualInputParam");    
    MySetSwitchThreshold = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetSwitchThreshold");
    MySetTorqueRateLimiter = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetTorqueRateLimiter");
    MySendJoystickCommand = (int (*)(JoystickCommand)) dlsym(commandLayer_handle,"Ethernet_SendJoystickCommand");
    MyStartControlAPI = (int (*)()) dlsym(commandLayer_handle,"Ethernet_StartControlAPI");
    MyGetJoystickValue = (int (*)(JoystickCommand &)) dlsym(commandLayer_handle,"Ethernet_GetJoystickValue");
    MyGetGripperStatus = (int (*)(Gripper &)) dlsym(commandLayer_handle,"Ethernet_GetGripperStatus");
    MyGetControlMapping = (int (*)(ControlMappingCharts &)) dlsym(commandLayer_handle,"Ethernet_GetControlMapping");
    // Verify that all essential functions have been loaded correctly
    if (
        MyInitAPI == NULL ||
        MyCloseAPI == NULL ||
        MyGetAngularCommand == NULL ||
        MyGetAngularPosition == NULL ||
        MyGetDevices == NULL ||
        MySetActiveDevice == NULL ||
        MyGetAngularVelocity == NULL ||
        MyGetAngularForce == NULL ||
        MyInitFingers == NULL ||
        MyGetAngularTorqueGravityEstimation == NULL || // Added verification for MyGetAngularTorqueGravityEstimation
        MySendJoystickCommand == NULL || 
        MyStartControlAPI == NULL
    )
    {
        std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << std::endl;
        // Optionally, you can print which function failed to load
        if (MyInitAPI == NULL) std::cout << "Failed to load MyInitAPI" << std::endl;
        if (MyCloseAPI == NULL) std::cout << "Failed to load MyCloseAPI" << std::endl;
        if (MyGetAngularCommand == NULL) std::cout << "Failed to load MyGetAngularCommand" << std::endl;
        if (MyGetAngularPosition == NULL) std::cout << "Failed to load MyGetAngularPosition" << std::endl;
        if (MyGetDevices == NULL) std::cout << "Failed to load MyGetDevices" << std::endl;
        if (MySetActiveDevice == NULL) std::cout << "Failed to load MySetActiveDevice" << std::endl;
        if (MyGetAngularVelocity == NULL) std::cout << "Failed to load MyGetAngularVelocity" << std::endl;
        if (MyGetAngularForce == NULL) std::cout << "Failed to load MyGetAngularForce" << std::endl;
        if (MyInitFingers == NULL) std::cout << "Failed to load MyInitFingers" << std::endl;
        if (MyGetAngularTorqueGravityEstimation == NULL) std::cout << "Failed to load MyGetAngularTorqueGravityEstimation" << std::endl;
        programResult = 0;
        return -1;
    }
    else
    {
        std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << std::endl << std::endl;
        // result = (*MyInitAPI)();
        EthernetCommConfig gEthernetCommConfig;
        gEthernetCommConfig.localBcastPort = 25015;
        gEthernetCommConfig.localCmdport = 25025;
        gEthernetCommConfig.robotPort = 55000;
        gEthernetCommConfig.localIpAddress = inet_addr("192.168.100.222"); // Your custom IP
        gEthernetCommConfig.robotIpAddress = inet_addr("192.168.100.19");
        gEthernetCommConfig.subnetMask = inet_addr("255.255.255.0");
        gEthernetCommConfig.rxTimeOutInMs = 10000;
        int initCommandResult = MyEthernet_InitEthernetAPI(&gEthernetCommConfig);
        if (initCommandResult != 1) { // Assuming 1 indicates success
            std::cerr << "Ethernet_InitAPI failed with error code: " << result << std::endl;
            programResult = 0;
            return -1;
 
        }
        else {
            
            AngularPosition DataCommand;
            // Get the angular command to test the communication with the robot
            resultComm = MyGetAngularCommand(DataCommand);
            std::cout << "Initialization's result: " << result << std::endl;
            std::cout << "Communication result: " << resultComm << std::endl;

            if (resultComm == 1)
            {

                std::cout << "API initialization worked" << std::endl;
                resultFingers = MyInitFingers();
                if (resultFingers == 1)
                {
                    std::cout << "Fingers initialized successfully" << std::endl;
                }
                else
                {
                    std::cout << "Failed to initialize fingers" << std::endl;
                }
                
                
                // Move to home position
                MyMoveHome();
                // Initialize fingers if necessary
                // MyInitFingers();
                
                resultControlAPI = MyStartControlAPI();
                virtualCommand.InitStruct();

                for (size_t i = 0; i < 1; i++)
                {
            
                    virtualCommand.ButtonValue[1] = 1;
                    //Every 1 ms we send a new virtual joystick command
                    //Button press release
                    for(int i = 0; i < 1000; i++)
                    {
                            MySendJoystickCommand(virtualCommand);
                            usleep(1000);

                    }

                    virtualCommand.ButtonValue[1] = 0;
                    
                    for(int i = 0; i < 1000; i++)
                    {
                            MySendJoystickCommand(virtualCommand);
                            usleep(1000);

                    }


                }

                for (size_t i = 0; i < 2; i++)
                {
                    std::cout << "Sending the command to the robotic arm." << std::endl;
                //Every 1 ms we send a new virtual joystick command
                    for(int i = 0; i < 500; i++)
                    {
                            virtualCommand.InclineLeftRight = 1;
                            MySendJoystickCommand(virtualCommand);
                            usleep(1000);
                    }
                    //Waiting 3 seconds
                    // virtualCommand.ButtonValue[1] = 0;
                    std::cout << "The robotic arm will now move backward from where it comes." << std::endl;
                    //we send the command backward
                    // virtualCommand.Rotate = -1;
                    std::cout << "Sending the command backward to the robotic arm." << std::endl;
                    //Every 1 ms we send a new virtual joystick command
                    for(int i = 0; i < 500; i++)
                    {
                            virtualCommand.InclineLeftRight = -1;
                            MySendJoystickCommand(virtualCommand);
                            usleep(1000);
                    }

                }
                
                std::cout << "The robot will switch to torque control mode and move. Be cautious." << std::endl;
                // Set Torque Robot Protection
                MySetTorqueRobotProtection(0);//2 is the maximum protection
                sleep(1);

                // MyEraseAllProtectionZones();
                // Move to home position
                MyMoveHome();

                //Move to the initial position
                moveRobotToStart();
                
                MySwitchTrajectoryTorque(POSITION);

                // Set the Switch Threshold to 0.1
                float SwitchThreshold[COMMAND_SIZE];
                for (int i = 0; i < COMMAND_SIZE; i++)
                {
                    SwitchThreshold[i] = 0;
                }
                SwitchThreshold[0] = 10;
                SwitchThreshold[1] = 15;
                SwitchThreshold[2] = 10;
                SwitchThreshold[3] = 5;
                MySetSwitchThreshold(SwitchThreshold);

                // Set the torque rate limiter to 0.1
                float TorqueRateLimiter[COMMAND_SIZE];
                for (int i = 0; i < COMMAND_SIZE; i++)
                {
                    TorqueRateLimiter[i] = 0;
                }
                TorqueRateLimiter[0] = 10;
                TorqueRateLimiter[1] = 10;
                TorqueRateLimiter[2] = 10;
                TorqueRateLimiter[3] = 10;
                MySetTorqueRateLimiter(TorqueRateLimiter);
                
                //Get gravity torque
                MyGetAngularTorqueGravityEstimation(gravityTorque);

                //Print the gravity torque
                std::cout << "Gravity Torque: " << std::endl;
                for (int i = 0; i < GRAVITY_PARAM_SIZE  ; i++)
                {
                    std::cout << gravityTorque[i] << " ";
                }
                std::cout << std::endl;
                /* // Make sure we use the gravity manual input type
                MySetGravityType(OPTIMAL);
                // Set the optimal Z parameter to 0
                float OptimalZParam[OPTIMAL_Z_PARAM_SIZE] = {0};
                MySetGravityOptimalZParam(OptimalZParam); */

                // Make sure we use the gravity manual input type
                /* MySetGravityType(MANUAL_INPUT);
                float ManualZParam[42] = {0};
                MySetGravityManualInputParam(ManualZParam); */
                // Set the optimal Z parameter to 0
                // Gravity vector in -X
                float GravityVector[3];
                GravityVector[0] = 0;
                GravityVector[1] = 0;
                GravityVector[2] = 0;
                // Set the gravity vector
                MySetGravityVector(GravityVector);
                
                //Reduce the damping
                // Set damping to 0.4
                float ActuatorDamping[COMMAND_SIZE];
                for (int i = 0; i < COMMAND_SIZE; i++)
                {
                        ActuatorDamping[i] = 0;
                }
                ActuatorDamping[0] = 1.5;
                ActuatorDamping[1] = 0.5;
                ActuatorDamping[2] = 0.5;
                ActuatorDamping[3] = 0.0;

                MySetTorqueActuatorDamping(ActuatorDamping);

                // Switch to torque control
                // (Here we switch before sending torques. The switch is possible because the gravity torques are already taken into account.)
                // MySwitchTrajectoryTorque(TORQUE);
                // Initialize the torque commands
                // float TorqueCommand[COMMAND_SIZE];
                // TorqueCommand[0] = 0.3;
                // TorqueCommand[1] = 8.5;
                // TorqueCommand[2] = -3.0;
                // TorqueCommand[3] = 0;
                // for (int i = 4; i < COMMAND_SIZE; i++)
                // {
                //     TorqueCommand[i] = 0;
                // }
                // // std::cout <<"Populating TorqueCommand" << std::endl;
                // // Send the torque command

                // MySendAngularTorqueCommand(TorqueCommand);

                // Set the torque control type to Direct Torque Control
                MySetTorqueControlType(DIRECTTORQUE);
                // Set the safety factor to 0.6
                MySetTorqueSafetyFactor(1.0f);
                // Set the vibration controller to 0.5
                MySetTorqueVibrationController(1.0f);

                /* MySetGravityType(MANUAL_INPUT);
                float ManualZParam[42] = {0};
                MySetGravityManualInputParam(ManualZParam); */

                (*MyClearErrorLog)();

                // Open a file to store torque data
                dataFile.open(filePath);
                if (!dataFile.is_open()) {
                    std::cerr << "Failed to open torque_data.txt for writing." << std::endl;
                    // Optionally, handle the error (e.g., exit or continue without logging)
                } else {
                    // Write headers
                    dataFile << "Time(ms) "
                             << "CmdTorque1 CmdTorque2 CmdTorque3 CmdTorque4 "
                             << "ActTorque1 ActTorque2 ActTorque3 ActTorque4 "
                             << "GravityTorque1 GravityTorque2 GravityTorque3 GravityTorque4\n";
                }
            }
            else
            {
                
                std::cout << "API initialization failed" << std::endl;
            }

        }   

    }
    return 1;
    

}

void KinovaArm::getPDTorques(Eigen::VectorXd& torqueCommand)
{


    double Kp[4] = {0.05, 0.05, 0.05, 0.02}; // Proportional gain for each actuator
    double Kd[4] = {0.01, 0.005, 0.005, 0.001}; // Derivative gain for each actuator
    double saturation[4] = {5.0, 5.0, 5.0, 4.0}; // Saturation for each actuator

    AngularPosition currentPositionKinova, currentVelocityKinova;
    //We get the actual angular position of the robot.
    MyGetAngularPosition(currentPositionKinova);
    //We get the actual angular velocity of the robot.
    MyGetAngularVelocity(currentVelocityKinova);


    double currentPosition[4] = {currentPositionKinova.Actuators.Actuator1,currentPositionKinova.Actuators.Actuator2,\
        currentPositionKinova.Actuators.Actuator3,currentPositionKinova.Actuators.Actuator4}; // Current position of each actuator
    double setPointActuator[4] = {startAngularPosition.Position.Actuators.Actuator1,startAngularPosition.Position.Actuators.Actuator2,\
        startAngularPosition.Position.Actuators.Actuator3,startAngularPosition.Position.Actuators.Actuator4}; // Setpoint for each actuator
    double currentVelocity[4] = {currentVelocityKinova.Actuators.Actuator1,currentVelocityKinova.Actuators.Actuator2,\
        currentVelocityKinova.Actuators.Actuator3,currentVelocityKinova.Actuators.Actuator4}; // Current velocity of each actuator

    /* if (currentPosition.Actuators.Actuator1 > 90)
   {
       setPointActuator1 = 270;
   }
   else if (currentPosition.Actuators.Actuator1 < 90)
   {
       currentPosition.Actuators.Actuator1 = -90;
   }
 */
   if (currentPositionKinova.Actuators.Actuator1 > 180)
   {
       setPointActuator[0] = 360;
   }
   else if (currentPositionKinova.Actuators.Actuator1 < -180)
   {
        setPointActuator[0] = -360;
   }
    
    
    // PD control algorithm
    for (int i = 0; i < 4; i++)
    {
        torqueCommand(i) = Kp[i] * (setPointActuator[i] - currentPosition[i]) + Kd[i] * (- currentVelocity[i]);
        // Apply saturation limits
        if (torqueCommand(i) > saturation[i])
        {
            torqueCommand(i) = saturation[i];
        }
        else if (torqueCommand(i) < -saturation[i])
        {
            torqueCommand(i) = -saturation[i];
        }
    }
    // Print the error for each actuator
    /* std::cout << "Error: ";
    for (int i = 0; i < 4; i++)
    {
        std::cout << "Actuator " << (i + 1) << ": " << setPointActuator[i] - currentPosition[i] << " ";
    }
    std::cout << std::endl;
     */
    std::cout << "Actuator: " << setPointActuator[1] - currentPosition[1] << "Current Position: " << currentPosition[1] << std::endl;

    //Print the torque command for each actuator
    /* std::cout << "Torque Command: ";
    for (int i = 0; i < 4; i++)
    {
        std::cout << "Actuator " << (i + 1) << ": " << torqueCommand[i] << " Nm ";

    }
    std::cout << endl; */
}

void KinovaArm::sendGripperCommand(float joystick_state[19]) {
    // Set the gripper command based on the triangle and square button values
    float triangle = joystick_state[15]; // Triangle button state
    float square = joystick_state[14]; // Square button state
    if (static_cast<bool>(triangle) && !static_cast<bool>(square) )
    {
        virtualCommand.InclineLeftRight = 1; // Move gripper to close position
    }
    else if (!static_cast<bool>(triangle) && static_cast<bool>(square))
    {
        virtualCommand.InclineLeftRight = -1; // Move gripper to open position
    }
    else
    {
        virtualCommand.InclineLeftRight = 0; // Stop gripper movement
    }    
    MyStartControlAPI();
    MySendJoystickCommand(virtualCommand);

    // Send the joystick command to the robot
}

void KinovaArm::runTorqueControl(Eigen::VectorXd jointTorqueKinova){

    
    //Check if it is in torque control mode
    MyGetTrajectoryTorqueMode(TrajectoryTorqueMode);
    std::cout << "TrajectoryTorqueMode: " << TrajectoryTorqueMode << std::endl;
    if(TrajectoryTorqueMode != 1)
    {
        MySwitchTrajectoryTorque(TORQUE);
    }
    

    //Get Torques from Shared Structs

    // Example: Set desired torques for all four joints
    // Customize these values as needed for your application
    /* TorqueCommand[0] = 0.8f;  // Joint 1 [MAX 5.0]
    TorqueCommand[1] = -1.4f; // Joint 2 [MAX 5.0]
    TorqueCommand[2] = 0.3f;  // Joint 3 [MAX 5.0]
    TorqueCommand[3] = 0.9f;  // Joint 4 [MAX 2.0]
     */
    /* TorqueCommand[0] = 0.0f;  // Joint 1 [MAX 5.0]
    TorqueCommand[1] = 0.0f; // Joint 2 [MAX 5.0]
    TorqueCommand[2] = 0.0f;  // Joint 3 [MAX 5.0]
    TorqueCommand[3] = 0.0f;  // Joint 4 [MAX 2.0] */

    TorqueCommand[0] = jointTorqueKinova(0);  // Joint 1 [MAX 5.0]
    TorqueCommand[1] = jointTorqueKinova(1); // Joint 2 [MAX 5.0]
    TorqueCommand[2] = jointTorqueKinova(2);  // Joint 3 [MAX 5.0]
    TorqueCommand[3] = jointTorqueKinova(3);  // Joint 4 [MAX 2.0]

   // If there are more joints, set their torques accordingly
    for(int j = 4; j < COMMAND_SIZE; j++) {
        TorqueCommand[j] = 0.0f; // Assuming 0 torque for additional joints
    }
    // MyGetTrajectoryTorqueMode(TrajectoryTorqueMode);
    // std::cout << "TrajectoryTorqueMode: " << TrajectoryTorqueMode << std::endl;

    // Send the torques
    MySendAngularTorqueCommand(TorqueCommand);
    
    /* 
    //Read the actual position and velocity
    int positionResult = MyGetAngularPosition(angularPosition);
    int velocityResult =  MyGetAngularVelocity(angularVelocity);
     */
    // Read the actual torque
    int torqueResult = MyGetAngularForce(actualTorque);

    
    // Initialize gravity torque array for 4 joints
    float gravityTorque[GRAVITY_PARAM_SIZE] = {0};

    if (torqueResult != 1) {
        std::cerr << "Failed to get actual torque at time " << motiontime << std::endl;
    } else {
        // Access the torque of all four joints
        float actualTorqueJoint1 = actualTorque.Actuators.Actuator1;
        float actualTorqueJoint2 = actualTorque.Actuators.Actuator2;
        float actualTorqueJoint3 = actualTorque.Actuators.Actuator3;
        float actualTorqueJoint4 = actualTorque.Actuators.Actuator4;
        // If there are more actuators, access them accordingly
        
        // Get the gravity torque estimations
        int gravityResult = MyGetAngularTorqueGravityEstimation(gravityTorque);
        if(gravityResult != 1) {
            std::cerr << "Failed to get gravity torque estimation at time in ms" << motiontime << std::endl;
        }

        /* QuickStatus sd;  
        int statusCheck = (*MyGetQuickStatus)(sd); */
        /* // Print to console
        std::cout << " Time(ms): " << motiontime 
                << " | CmdTorque1: " << TorqueCommand[0]
                << " Nm | CmdTorque2: " << TorqueCommand[1]
                << " Nm | CmdTorque3: " << TorqueCommand[2]
                << " Nm | CmdTorque4: " << TorqueCommand[3]
                << " Nm | ActTorque1: " << actualTorqueJoint1
                << " Nm | ActTorque2: " << actualTorqueJoint2
                << " Nm | ActTorque3: " << actualTorqueJoint3
                << " Nm | ActTorque4: " << actualTorqueJoint4
                << " Nm | ForceControlStatus: " << (int)sd.ForceControlStatus
                << std::endl;
        
        // Additionally, print gravity torque estimations
        std::cout << "Gravity Torque Estimations: ";
        for(int g = 0; g < GRAVITY_PARAM_SIZE; g++) {
            std::cout << "GT" << (g+1) << ": " << gravityTorque[g] << " Nm ";
        }
        std::cout << std::endl;
 */
        // Log to file if opened
        if (dataFile.is_open()) {
            dataFile << motiontime << " " 
                        << TorqueCommand[0] << " " 
                        << TorqueCommand[1] << " " 
                        << TorqueCommand[2] << " " 
                        << TorqueCommand[3] << " "
                        << actualTorqueJoint1 << " " 
                        << actualTorqueJoint2 << " " 
                        << actualTorqueJoint3 << " " 
                        << actualTorqueJoint4 << " ";
            for(int g = 0; g < GRAVITY_PARAM_SIZE; g++) {
                dataFile << gravityTorque[g] << " ";
            }
            dataFile << "\n";
        }
    }
    //Set Angular Position and Velocities to Shared Structs
    motiontime += 16.7; //Assuming 10 ms for each iteration

    /* resultError = (*MyGetSystemErrorCount)(ErrorCount);
    std::cout << "System error count : " << ErrorCount << std::endl << std::endl << std::endl;
    for(unsigned int i = 0; i < ErrorCount; i++)
        {
            result = (*MyGetSystemError)(i, errorTemp);
            
            std::cout << "System error #" << i << " type : " << errorTemp.ErrorType << std::endl;
        } */
}

void KinovaArm::holdPosition(){
    //Check if it is in torque control mode
    MyGetTrajectoryTorqueMode(TrajectoryTorqueMode);
    if(TrajectoryTorqueMode == 1)
    {
        MySwitchTrajectoryTorque(POSITION);
    }
}

void KinovaArm::cleanUpKinova(){
    // Switch back to position mode
    MySwitchTrajectoryTorque(POSITION);

    // Close the data file
    if (dataFile.is_open()) {
        dataFile.close();
        std::cout << "Torque data and Gravity Estimation logged to torque_data.txt" << std::endl;
    }
    std::cout << std::endl << "C L O S I N G   A P I" << std::endl;
    result = (*MyCloseAPI)();
    programResult = 1;
    dlclose(commandLayer_handle);
}

void KinovaArm::setStartPose(double angular_pose[4]){
    startAngularPosition.InitStruct();
    startAngularPosition.Position.Type = ANGULAR_POSITION;
    startAngularPosition.Position.HandMode = HAND_NOMOVEMENT;  
    

    startAngularPosition.Position.Actuators.Actuator1 = angular_pose[0];//0;//currentPosition.Actuators.Actuator1+60;
    startAngularPosition.Position.Actuators.Actuator2 = angular_pose[1];//currentPosition.Actuators.Actuator2;
    startAngularPosition.Position.Actuators.Actuator3 = angular_pose[2];//currentPosition.Actuators.Actuator3;
    startAngularPosition.Position.Actuators.Actuator4 = angular_pose[3];// currentPosition.Actuators.Actuator4;
    startAngularPosition.LimitationsActive = 1;
    startAngularPosition.Limitations.speedParameter1 = 25;
    startAngularPosition.Limitations.speedParameter2 = 25;
}

void KinovaArm::moveRobotToStart(){
    
    AngularPosition currentCommand;
    AngularPosition currentPosition;
    //We get the actual angular position of the robot.
    MyGetAngularPosition(currentPosition);
    
    std::cout << "Actuator 1 current position : " << currentPosition.Actuators.Actuator1 << "°" << std::endl;
    std::cout << "Actuator 2 current position : " << currentPosition.Actuators.Actuator2 << "°" << std::endl;
    std::cout << "Actuator 3 current position : " << currentPosition.Actuators.Actuator3 << "°" << std::endl;
    std::cout << "Actuator 4 current position : " << currentPosition.Actuators.Actuator4 << "°" << std::endl;

   /* if (currentPosition.Actuators.Actuator1 > 180)
   {
       startAngularPosition.Position.Actuators.Actuator1 = 360;
   }
   else if (currentPosition.Actuators.Actuator1 < -180)
   {
    startAngularPosition.Position.Actuators.Actuator1= -360;
   } */
   
    std::cout << "Actuator 1 desired position : " << startAngularPosition.Position.Actuators.Actuator1 << "°" << std::endl;
    std::cout << "Actuator 2 desired position : " << startAngularPosition.Position.Actuators.Actuator2 << "°" << std::endl;
    std::cout << "Actuator 3 desired position : " << startAngularPosition.Position.Actuators.Actuator3 << "°" << std::endl;
    std::cout << "Actuator 4 desired position : " << startAngularPosition.Position.Actuators.Actuator4 << "°" << std::endl;
    
    float P = 1.0;
    float I = 0;
    float D = 0.02;
    /* float P = 2.0;
    float I = 0;
    float D = 0.05; */
    unsigned int ActuatorAddress1 = 16;
    unsigned int ActuatorAddress2 = 17;
    unsigned int ActuatorAddress3 = 18;
    unsigned int ActuatorAddress4 = 19;
    MySetActuatorPID(ActuatorAddress1, P, I, D);
    MySetActuatorPID(ActuatorAddress2, P, I, D);
    MySetActuatorPID(ActuatorAddress3, P, I, D);
    MySetActuatorPID(ActuatorAddress4, P, I, D);

    std::cout << "*********************************" << std::endl;
    std::cout << "Moving robot to pose" << std::endl;
    MySendAdvanceTrajectory(startAngularPosition);
    sleep(15);
    std::cout << "Robot moved to pose" << std::endl;
    std::cout << "*********************************" << std::endl;
    // We get the actual angular position of the robot.
    MyGetAngularPosition(currentPosition);
    std::cout << "Actuator 1 current position : " << currentPosition.Actuators.Actuator1 << "°" << std::endl;
    std::cout << "Actuator 2 current position : " << currentPosition.Actuators.Actuator2 << "°" << std::endl;
    std::cout << "Actuator 3 current position : " << currentPosition.Actuators.Actuator3 << "°" << std::endl;
    std::cout << "Actuator 4 current position : " << currentPosition.Actuators.Actuator4 << "°" << std::endl;

}

void KinovaArm::getAngularPosition(double angular_pose[4]){
    
    MyGetAngularPosition(angularPosition);

    angular_pose[0] = (angularPosition.Actuators.Actuator1/180.0)*M_PI;
    angular_pose[1] = (angularPosition.Actuators.Actuator2/180.0)*M_PI;
    angular_pose[2] = (angularPosition.Actuators.Actuator3/180.0)*M_PI;
    angular_pose[3] = (angularPosition.Actuators.Actuator4/180.0)*M_PI;

}

void KinovaArm::getAngularVelocity(double angular_velocity[4]){
    
    MyGetAngularVelocity(angularVelocity);

    angular_velocity[0] = (angularVelocity.Actuators.Actuator1/180.0)*M_PI;
    angular_velocity[1] = (angularVelocity.Actuators.Actuator2/180.0)*M_PI;
    angular_velocity[2] = (angularVelocity.Actuators.Actuator3/180.0)*M_PI;
    angular_velocity[3] = (angularVelocity.Actuators.Actuator4/180.0)*M_PI;

}