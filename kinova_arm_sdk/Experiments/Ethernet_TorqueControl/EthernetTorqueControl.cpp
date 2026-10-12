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

using namespace std;

// Define constants if not defined in KinovaTypes.h
#ifndef COMMAND_SIZE
#define COMMAND_SIZE 7 // Adjust according to your Kinova model
#endif

//#ifndef GRAVITY_PARAM_SIZE
#define GRAVITY_PARAM_SIZE 4 // Adjust to 4 since your robot has 4 joints
//#endif

int main()
{
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
    int(*MySetGravityType)(GRAVITY_TYPE Type);
    int(*MySetTorqueVibrationController)(float value);
    int(*MyGetAngularForceGravityFree)(AngularPosition &);
    int(*MyGetCartesianForce)(CartesianPosition &);
    int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
    int(*MyGetAngularForce)(AngularPosition & Response ); // Corrected function pointer
    int(*MyGetQuickStatus)( QuickStatus & Response); 	
    int(*MyGetAngularTorqueGravityEstimation)(float Command[GRAVITY_PARAM_SIZE]);

    // We load the library (Under Windows, use the function LoadLibrary)
    commandLayer_handle = dlopen("Kinova.API.EthCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
    if (!commandLayer_handle) {
        cerr << "Failed to load Kinova.API.EthCommandLayerUbuntu.so: " << dlerror() << endl;
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
    int (*MySetEthernetConfiguration)(EthernetConfiguration* config) = (int (*)(EthernetConfiguration * config)) dlsym(commandLayer_handle, "Ethernet_SetEthernetConfiguration");
    int (*MyGetEthernetConfiguration)(EthernetConfiguration* config) = (int (*)(EthernetConfiguration * config)) dlsym(commandLayer_handle, "Ethernet_GetEthernetConfiguration");
    int (*MyEthernet_InitEthernetAPI)(EthernetCommConfig* config) = (int (*)(EthernetCommConfig * config)) dlsym(commandLayer_handle, "Ethernet_InitEthernetAPI");
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
        MyGetAngularTorqueGravityEstimation == NULL // Added verification for MyGetAngularTorqueGravityEstimation
    )
    {
        cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
        // Optionally, you can print which function failed to load
        if (MyInitAPI == NULL) cout << "Failed to load MyInitAPI" << endl;
        if (MyCloseAPI == NULL) cout << "Failed to load MyCloseAPI" << endl;
        if (MyGetAngularCommand == NULL) cout << "Failed to load MyGetAngularCommand" << endl;
        if (MyGetAngularPosition == NULL) cout << "Failed to load MyGetAngularPosition" << endl;
        if (MyGetDevices == NULL) cout << "Failed to load MyGetDevices" << endl;
        if (MySetActiveDevice == NULL) cout << "Failed to load MySetActiveDevice" << endl;
        if (MyGetAngularVelocity == NULL) cout << "Failed to load MyGetAngularVelocity" << endl;
        if (MyGetAngularForce == NULL) cout << "Failed to load MyGetAngularForce" << endl;
        if (MyInitFingers == NULL) cout << "Failed to load MyInitFingers" << endl;
        if (MyGetAngularTorqueGravityEstimation == NULL) cout << "Failed to load MyGetAngularTorqueGravityEstimation" << endl;
        programResult = 0;
    }
    else
    {
        cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
        // result = (*MyInitAPI)();
        EthernetCommConfig gEthernetCommConfig;
        gEthernetCommConfig.localBcastPort = 25015;
        gEthernetCommConfig.localCmdport = 25025;
        gEthernetCommConfig.robotPort = 55000;
        gEthernetCommConfig.localIpAddress = inet_addr("192.168.123.101"); // Your custom IP
        gEthernetCommConfig.robotIpAddress = inet_addr("192.168.123.19");
        gEthernetCommConfig.subnetMask = inet_addr("255.255.255.0");
        gEthernetCommConfig.rxTimeOutInMs = 1000;
        int initCommandResult = MyEthernet_InitEthernetAPI(&gEthernetCommConfig);
        if (initCommandResult != 1) { // Assuming 1 indicates success
            cerr << "Ethernet_InitAPI failed with error code: " << result << endl;
            programResult = 0;
        }
        else {
            int resultComm;
            AngularPosition DataCommand;
            // Get the angular command to test the communication with the robot
            resultComm = MyGetAngularCommand(DataCommand);
            cout << "Initialization's result: " << result << endl;
            cout << "Communication result: " << resultComm << endl;
            // If the API is initialized and the communication with the robot is working
            if (resultComm == 1)
            {
                cout << "API initialization worked" << endl;
                cout << "The robot will switch to torque control mode and move. Be cautious." << endl;
                // Set to position mode
                MySwitchTrajectoryTorque(POSITION);
                // Move to home position
                MyMoveHome();
                // Initialize fingers if necessary
                MyInitFingers();
                // Set the torque control type to Direct Torque Control
                MySetTorqueControlType(DIRECTTORQUE);
                // Set the safety factor to 0.6
                MySetTorqueSafetyFactor(0.6f);
                // Set the vibration controller to 0.5
                MySetTorqueVibrationController(0.5f);
                // Switch to torque control
                // (Here we switch before sending torques. The switch is possible because the gravity torques are already taken into account.)
                MySwitchTrajectoryTorque(TORQUE);
                // Initialize the torque commands
                float TorqueCommand[COMMAND_SIZE];
                for (int i = 0; i < COMMAND_SIZE; i++)
                {
                    TorqueCommand[i] = 0;
                }

                // Open a file to store torque data
                ofstream dataFile("torque_data.txt");
                if (!dataFile.is_open()) {
                    cerr << "Failed to open torque_data.txt for writing." << endl;
                    // Optionally, handle the error (e.g., exit or continue without logging)
                } else {
                    // Write headers
                    dataFile << "Iteration Time(ms) "
                             << "CmdTorque1 CmdTorque2 CmdTorque3 CmdTorque4 "
                             << "ActTorque1 ActTorque2 ActTorque3 ActTorque4 "
                             << "GravityTorque1 GravityTorque2 GravityTorque3 GravityTorque4\n";
                }

                // Send the torque commands for a specified duration
                // Assuming usleep(10000) sleeps for 10 ms, loop accordingly
                int totalIterations = 500; // For 500 * 10ms = 5000ms = 5 seconds
                for (int i = 0; i < totalIterations; i++)
                {
                    // Example: Set desired torques for all four joints
                    // Customize these values as needed for your application
                    TorqueCommand[0] = 0.8f;  // Joint 1 [MAX 5.0]
                    TorqueCommand[1] = -1.4f; // Joint 2 [MAX 5.0]
                    TorqueCommand[2] = 0.3f;  // Joint 3 [MAX 5.0]
                    TorqueCommand[3] = 0.9f;  // Joint 4 [MAX 2.0]
                    // If there are more joints, set their torques accordingly
                    for(int j = 4; j < COMMAND_SIZE; j++) {
                        TorqueCommand[j] = 0.0f; // Assuming 0 torque for additional joints
                    }

                    // Send the torques
                    MySendAngularTorqueCommand(TorqueCommand);
                    
                    // Read the actual torque
                    AngularPosition actualTorque;
                    int torqueResult = MyGetAngularForce(actualTorque);

                    // Initialize gravity torque array for 4 joints
                    float gravityTorque[GRAVITY_PARAM_SIZE] = {0};

                    if (torqueResult != 1) {
                        cerr << "Failed to get actual torque at iteration " << i << endl;
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
                            cerr << "Failed to get gravity torque estimation at iteration " << i << endl;
                        }

                        QuickStatus sd;  
                        int statusCheck = (*MyGetQuickStatus)(sd);
                        // Print to console
                        cout << "Iteration: " << i 
                             << " | Time(ms): " << i * 10 
                             << " | CmdTorque1: " << TorqueCommand[0]
                             << " Nm | CmdTorque2: " << TorqueCommand[1]
                             << " Nm | CmdTorque3: " << TorqueCommand[2]
                             << " Nm | CmdTorque4: " << TorqueCommand[3]
                             << " Nm | ActTorque1: " << actualTorqueJoint1
                             << " Nm | ActTorque2: " << actualTorqueJoint2
                             << " Nm | ActTorque3: " << actualTorqueJoint3
                             << " Nm | ActTorque4: " << actualTorqueJoint4
                             << " Nm | ForceControlStatus: " << (int)sd.ForceControlStatus
                             << endl;
                        
                        // Additionally, print gravity torque estimations
                        cout << "Gravity Torque Estimations: ";
                        for(int g = 0; g < GRAVITY_PARAM_SIZE; g++) {
                            cout << "GT" << (g+1) << ": " << gravityTorque[g] << " Nm ";
                        }
                        cout << endl;

                        // Log to file if opened
                        if (dataFile.is_open()) {
                            dataFile << i << " " 
                                     << (i * 10) << " " 
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
                    // Sleep 10 ms
                    usleep(10000);
                }
                
                // Switch back to position mode
                MySwitchTrajectoryTorque(POSITION);

                // Close the data file
                if (dataFile.is_open()) {
                    dataFile.close();
                    cout << "Torque data and Gravity Estimation logged to torque_data.txt" << endl;
                }
            }
            else
            {
                cout << "API initialization failed" << endl;
            }
            cout << endl << "C L O S I N G   A P I" << endl;
            result = (*MyCloseAPI)();
            programResult = 1;
        }
        dlclose(commandLayer_handle);
        return programResult;
    } // <-- Ensure this closing brace exists
}