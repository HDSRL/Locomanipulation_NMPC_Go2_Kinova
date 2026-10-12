// Vittorio Pastore
// HDSRL
// November 7, 2024 
// Modified on December 3, 2024
// Test code to verify connection to the Kinova robot arm over Ethernet with multithreading for sensor readings

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#ifdef __linux__ 
#include <dlfcn.h>
#include "Kinova.API.EthCommLayerUbuntu.h"
#include "Kinova.API.EthCommandLayerUbuntu.h"
#include <unistd.h>
#endif

using namespace std;

// Function pointers for Kinova API functions
#ifdef __linux__
typedef int (*InitAPIFunc)();
typedef int (*CloseAPIFunc)();
typedef int (*GetDevicesFunc)(KinovaDevice[MAX_KINOVA_DEVICE], int &);
typedef int (*RefreshDevicesListFunc)();
typedef int (*GetCartesianPositionFunc)(CartesianPosition *);
typedef int (*MoveHomeFunc)();
typedef int (*SetCartesianControlFunc)();
#endif

int main()
{
#ifdef __linux__ 
    // Load the command layer library
    void* commandLayer_handle = dlopen("Kinova.API.EthCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
    if (!commandLayer_handle) {
        cerr << "Error loading Kinova command layer library: " << dlerror() << endl;
        return -1;
    }

    // Load functions from the library
    InitAPIFunc MyInitAPI = (InitAPIFunc) dlsym(commandLayer_handle, "Ethernet_InitAPI");
    CloseAPIFunc MyCloseAPI = (CloseAPIFunc) dlsym(commandLayer_handle, "Ethernet_CloseAPI");
    GetDevicesFunc MyGetDevices = (GetDevicesFunc) dlsym(commandLayer_handle, "Ethernet_GetDevices");
    RefreshDevicesListFunc MyRefreshDevicesList = (RefreshDevicesListFunc) dlsym(commandLayer_handle, "Ethernet_RefresDevicesList");
    GetCartesianPositionFunc MyGetCartesianPosition = (GetCartesianPositionFunc) dlsym(commandLayer_handle, "Ethernet_GetCartesianPosition");
    MoveHomeFunc MyMoveHome = (MoveHomeFunc) dlsym(commandLayer_handle, "Ethernet_MoveHome");
    SetCartesianControlFunc MySetCartesianControl = (SetCartesianControlFunc) dlsym(commandLayer_handle, "Ethernet_SetCartesianControl");
#endif

    // Verify all functions are loaded
    if (
        !MyInitAPI || 
        !MyCloseAPI || 
        !MyGetDevices || 
        !MyRefreshDevicesList || 
        !MyGetCartesianPosition || 
        !MyMoveHome || 
        !MySetCartesianControl
    ) {
        cerr << "Error resolving one or more functions from library." << endl;
#ifdef __linux__
        dlclose(commandLayer_handle);
#endif
        return -1;
    }

    // Initialize API
    cout << "Initializing API..." << endl;
    int initResult = MyInitAPI();
    if (initResult != 1) {
        cerr << "Failed to initialize the API. Return code: " << initResult << endl;
        return -1;
    } else {
        cout << "API initialized successfully." << endl;
    }

    // Refresh device list
    cout << "Refreshing devices list..." << endl;
    MyRefreshDevicesList();

    // Get devices and check if any are available
    KinovaDevice devices[MAX_KINOVA_DEVICE];
    int deviceResult;
    int devicesCount = MyGetDevices(devices, deviceResult);

    if (devicesCount > 0) {
        cout << "Robot connected! Debug message: Robot with Serial Number " 
             << devices[0].SerialNumber << " is online." << endl;

        cout << "Moving to home position..." << endl;
        MyMoveHome();

        cout << "Setting Cartesian control..." << endl;
        MySetCartesianControl();

        // Atomic flags for thread control
        atomic<bool> keepReading(true);       // Flag to keep the sensor thread running
        atomic<bool> isReading(true);         // Flag to pause/resume sensor reading

        // Mutex and condition variable for synchronizing pause/resume
        mutex mtx;
        condition_variable cv;

        // Sensor reading thread function
        auto sensorThreadFunc = [&]() {
            CartesianPosition position;
            while (keepReading.load()) {
                // Wait if reading is paused
                {
                    unique_lock<mutex> lock(mtx);
                    cv.wait(lock, [&]() { return isReading.load() || !keepReading.load(); });
                    if (!keepReading.load()) break;
                }

                // Get Cartesian position
                int result = MyGetCartesianPosition(&position);

                if (result == 1) {
                    cout << "X: " << position.Coordinates.X << " m, "
                         << "Y: " << position.Coordinates.Y << " m, "
                         << "Z: " << position.Coordinates.Z << " m" << endl;
                } else {
                    cerr << "Failed to get Cartesian position." << endl;
                }

                // Sleep for 500 ms
                this_thread::sleep_for(chrono::milliseconds(500));
            }
            cout << "Sensor reading thread exiting..." << endl;
        };

        // Start sensor reading thread
        thread sensorThread(sensorThreadFunc);
        cout << "Sensor reading thread started." << endl;

        // Main thread handles user input
        cout << "Enter commands to control sensor reading:" << endl;
        cout << "  'e' - Exit the program" << endl;
        cout << "  'p' - Pause sensor reading" << endl;
        cout << "  'r' - Resume sensor reading" << endl;

        while (true) {
            char command;
            cout << "Enter command: ";
            cin >> command;

            if (command == 'e' || command == 'E') {
                cout << "Exiting program..." << endl;
                keepReading.store(false);
                // Notify sensor thread in case it's waiting
                cv.notify_all();
                break;
            }
            else if (command == 'p' || command == 'P') {
                if (isReading.load()) {
                    isReading.store(false);
                    cout << "Sensor reading paused." << endl;
                } else {
                    cout << "Sensor reading is already paused." << endl;
                }
            }
            else if (command == 'r' || command == 'R') {
                if (!isReading.load()) {
                    isReading.store(true);
                    cv.notify_all(); // Wake up the sensor thread
                    cout << "Sensor reading resumed." << endl;
                } else {
                    cout << "Sensor reading is already running." << endl;
                }
            }
            else {
                cout << "Unknown command. Please enter 'e', 'p', or 'r'." << endl;
            }
        }

        // Wait for the sensor thread to finish
        if (sensorThread.joinable()) {
            sensorThread.join();
        }

    } else {
        cout << "No robot found on the network. Devices count: " << devicesCount << endl;
    }

    // Close API
    cout << "Closing API..." << endl;
    MyCloseAPI();

#ifdef __linux__ 
    dlclose(commandLayer_handle);
#endif

    cout << "Program terminated successfully." << endl;
    return 0;
}

