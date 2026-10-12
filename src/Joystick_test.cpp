#include "JoyShockLibrary.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <ctime>
#include <unistd.h>

bool isButtonPressed(int buttons, int buttonMask) {
    return (buttons & buttonMask) != 0;
}

int main() {

    int deviceCount = JslConnectDevices();

    if (deviceCount <= 0) {
        std::cerr << "No devices found." << std::endl;
        return -1;  
    }

    int deviceHandles[deviceCount];
    int numDevices = JslGetConnectedDeviceHandles(deviceHandles, deviceCount);
    if (numDevices <= 0) {
        std::cerr << "No devices connected." << std::endl;
        return -1;  
    }
    std::cout << "Connected devices: " << numDevices << std::endl;
    for (int i = 0; i < numDevices; ++i) {
        std::cout << "Device " << i << ": Handle " << deviceHandles[i] << std::endl;
    }
    // Wait for devices to initialize
    std::cout << "Waiting for devices to initialize..." << std::endl;
    sleep(1); // Wait for devices to initialize

    JOY_SHOCK_STATE state;
    IMU_STATE imuState;
    TOUCH_STATE touchState;
    JSL_AUTO_CALIBRATION autoCalibration;
    MOTION_STATE motionState;
    int deviceId = deviceHandles[0]; // Use the first connected device
    int gyroSpace = 0;
    int colour = 0;
    int playerNumber = 0;
    int splitType = 0;
    bool isCalibrating = false;
    bool autoCalibrationEnabled = false;
    bool isConnected = false;
    while (true) {
        // Get the current state of the joystick
        state = JslGetSimpleState(deviceId);
        imuState = JslGetIMUState(deviceId);
        touchState = JslGetTouchState(deviceId);


        // Print joystick state
        std::cout << "Buttons: " << state.buttons
        << ", LStickX: " << state.stickLX
        << ", LStickY: " << state.stickLY
        << ", RStickX: " << state.stickRX
            << ", RStickY: " << state.stickRY << std::endl;

        /* // Print IMU state
        IMU_STATE imuState = JslGetIMUState(deviceId);
        std::cout << "GyroX: " << imuState.gyroX
            << ", GyroY: " << imuState.gyroY
            << ", GyroZ: " << imuState.gyroZ
            << ", AccelX: " << imuState.accelX
            << ", AccelY: " << imuState.accelY
            << ", AccelZ: " << imuState.accelZ << std::endl; */

        /* // Print touch state (if supported)
        TOUCH_STATE touchState = JslGetTouchState(deviceId);
        std::cout << "Touch 1: (" << touchState.t0Id << ", " << touchState.t0X
            << ", " << touchState.t0Y << "), Touch 2: (" << touchState.t1Id
            << ", " << touchState.t1X << ", " << touchState.t1Y << ")" << std::endl; */

        // Print button states individually using bitwise checking
        std::cout << "D-Pad Up: " << (isButtonPressed(state.buttons, 0x00001) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "D-Pad Down: " << (isButtonPressed(state.buttons, 0x00002) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "D-Pad Left: " << (isButtonPressed(state.buttons, 0x00004) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "D-Pad Right: " << (isButtonPressed(state.buttons, 0x00008) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "L1: " << (isButtonPressed(state.buttons, 0x00100) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "R1: " << (isButtonPressed(state.buttons, 0x00200) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "L2: " << (isButtonPressed(state.buttons, 0x00400) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "R2: " << (isButtonPressed(state.buttons, 0x00800) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "⨉:" << (isButtonPressed(state.buttons, 0x01000) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "○: " << (isButtonPressed(state.buttons, 0x02000) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << " □: " << (isButtonPressed(state.buttons, 0x04000) ? "Pressed" : "Not Pressed") << std::endl;
        std::cout << "△: " << (isButtonPressed(state.buttons, 0x08000) ? "Pressed" : "Not Pressed") << std::endl;

        sleep(1); // Sleep for a short duration to avoid overwhelming the console output
    }


    // Terminate the connection
    JslDisconnectAndDisposeAll();


    


    return 0;
}
