#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP


#include "JoyShockLibrary.h"
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <fstream>  // For file operations
#include <vector>   // For storing data
#include <string>   // For string operations

#define J_DPAD_UP 0x0001
#define J_DPAD_DOWN 0x0002
#define J_DPAD_LEFT 0x0004
#define J_DPAD_RIGHT 0x0008
#define J_L1 0x00100
#define J_R1 0x00200
#define J_L2 0x00400
#define J_R2 0x00800
#define J_CROSS 0x01000
#define J_CIRCLE 0x02000
#define J_SQUARE 0x04000
#define J_TRIANGLE 0x08000
#define J_SHARE 0x00020
#define J_OPTIONS 0x00010
#define J_HOME 0x10000

class JoyStick {
    private:
        int deviceCount;
        int deviceHandles[10];
        int numDevices;
        bool isConnected = false;
        int joystickNumber = 0;
        int buttons;
        JOY_SHOCK_STATE joyShockState;
        IMU_STATE imuState;
        TOUCH_STATE touchState;
        JSL_AUTO_CALIBRATION autoCalibration;
        MOTION_STATE motionState;
        int deviceId = 0;
        int gyroSpace = 0;
        int colour = 0;
        float LStickX = 0.f;
        float LStickY = 0.f;
        float RStickX = 0.f;
        float RStickY = 0.f;
        bool D_padUp = false;
        bool D_padDown = false;
        bool D_padLeft = false;
        bool D_padRight = false;
        bool L1 = false;
        bool L2 = false;
        bool R1 = false;
        bool R2 = false;
        bool cross = false;
        bool circle = false;
        bool square = false;
        bool triangle = false;
        bool share = false;
        bool options = false;
        bool home = false;
        float joystickState[19] = {0}; //Buttons we want to track
        
    public:
        
        JoyStick(){
           initJoyStick();
        };

        ~JoyStick(){
            disconnectJoyStick();
        };

        void initJoyStick();
        void disconnectJoyStick();
        bool isButtonPressed(int buttonMask);
        void getJoyShockState(float joystickState[19]);
        void printJoyShockState();
};


#endif // JOYSTICK_HPP

