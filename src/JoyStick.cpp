#include "JoyStick.hpp"
#include <unistd.h>
#include "JoyShockLibrary.h"

void JoyStick::initJoyStick(){
    deviceCount = JslConnectDevices();

    if (deviceCount <= 0) {
        std::cerr << "No devices found." << std::endl;

    }


    numDevices = JslGetConnectedDeviceHandles(deviceHandles, deviceCount);
    if (numDevices <= 0) {
        std::cerr << "No devices connected." << std::endl;

    }
    std::cout << "Connected devices: " << numDevices << std::endl;
    for (int i = 0; i < numDevices; ++i) {
        std::cout << "Device " << i << ": Handle " << deviceHandles[i] << std::endl;
    }
    // Wait for devices to initialize
    std::cout << "Waiting for devices to initialize..." << std::endl;
    sleep(1); // Wait for devices to initialize
}



void JoyStick::disconnectJoyStick(){
    JslDisconnectAndDisposeAll();
    std::cout << "Disconnected all devices." << std::endl;
}

bool JoyStick::isButtonPressed(int buttonMask) {
    return (buttons & buttonMask) != 0;
}


void JoyStick::getJoyShockState(float joystickState[19]){
    // Check if the device is connected
    // Check if the device is still connected
    if (!JslStillConnected(deviceId)) {
        std::cerr << "Device disconnected." << std::endl;
        return;
    }
    // Get the current state of the joystick
    joyShockState = JslGetSimpleState(deviceId);
    buttons = joyShockState.buttons;
    LStickX = joyShockState.stickLX;
    LStickY = joyShockState.stickLY;
    RStickX = joyShockState.stickRX;
    RStickY = joyShockState.stickRY;
    D_padUp = isButtonPressed(J_DPAD_UP);
    D_padDown = isButtonPressed(J_DPAD_DOWN);
    D_padLeft = isButtonPressed(J_DPAD_LEFT);
    D_padRight = isButtonPressed(J_DPAD_RIGHT);
    L1 = isButtonPressed(J_L1);
    L2 = isButtonPressed(J_L2);
    R1 = isButtonPressed(J_R1);
    R2 = isButtonPressed(J_R2);
    cross = isButtonPressed(J_CROSS);
    circle = isButtonPressed(J_CIRCLE);
    square = isButtonPressed(J_SQUARE);
    triangle = isButtonPressed(J_TRIANGLE);
    share = isButtonPressed(J_SHARE);
    options = isButtonPressed(J_OPTIONS);
    home = isButtonPressed(J_HOME);
    
    //Update joystickState array
    joystickState[0] = -LStickX;
    joystickState[1] = LStickY;
    joystickState[2] = -RStickX;
    joystickState[3] = RStickY;
    joystickState[4] = D_padUp ? 1.0f : (D_padDown ? -1.0f : 0.0f);
    joystickState[5] = 0;
    joystickState[6] = D_padLeft ? -1.0f : (D_padRight ? 1.0f : 0.0f);
    joystickState[7] = 0;
    joystickState[8] = L1 ? 1.0f : (L2 ? -1.0f : 0.0f);  
    joystickState[9] = 0;
    joystickState[10] = R1 ? 1.0f : (R2 ? -1.0f : 0.0f);
    joystickState[11] = 0;
    joystickState[12] = static_cast<float>(cross);
    joystickState[13] = static_cast<float>(circle);
    joystickState[14] = static_cast<float>(square);
    joystickState[15] = static_cast<float>(triangle);
    joystickState[16] = static_cast<float>(share);
    joystickState[17] = static_cast<float>(options);
    joystickState[18] = static_cast<float>(home);


}


void JoyStick::printJoyShockState(){
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Sticks: " << buttons
    << ", LStickX: " << LStickX
    << ", LStickY: " << LStickY
    << ", RStickX: " << RStickX
    << ", RStickY: " << RStickY << std::endl;
    std::cout << "D-pad Up: " << D_padUp
    << ", D-pad Down: " << D_padDown
    << ", D-pad Left: " << D_padLeft
    << ", D-pad Right: " << D_padRight << std::endl;
    std::cout << "L1: " << L1
    << ", L2: " << L2
    << ", R1: " << R1
    << ", R2: " << R2 << std::endl;
    std::cout << "Cross: " << cross
    << ", Circle: " << circle
    << ", Square: " << square
    << ", Triangle: " << triangle << std::endl;
    std::cout << "Share: " << share
    << ", Options: " << options << std::endl;
    std::cout << "Home: " << home << std::endl;
    std::cout << "----------------------------------------" << std::endl;
}


