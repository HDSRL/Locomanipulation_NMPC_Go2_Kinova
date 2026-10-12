// Vittorio Pastore
// HDSRL
// November 7, 2024 
// Simple test code to verify connection to the Kinova robot arm over Ethernet and display a debug message

#include <iostream>
#include "KinovaTypes.h"
#ifdef __linux__ 
#include <dlfcn.h>
#include "Kinova.API.EthCommLayerUbuntu.h"
#include "Kinova.API.EthCommandLayerUbuntu.h"
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#include "CommandLayer.h"
#include "CommunicationLayer.h"
#endif

using namespace std;

int main()
{
#ifdef __linux__ 
    void* commandLayer_handle = dlopen("Kinova.API.EthCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
    if (!commandLayer_handle) {
        cerr << "Error loading Kinova command layer library: " << dlerror() << endl;
        return -1;
    }

    // Load functions from the library
    int (*MyInitAPI)() = (int (*)()) dlsym(commandLayer_handle, "Ethernet_InitAPI");
    int (*MyCloseAPI)() = (int (*)()) dlsym(commandLayer_handle, "Ethernet_CloseAPI");
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) =
        (int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int&)) dlsym(commandLayer_handle, "Ethernet_GetDevices");
    int (*MyRefresDevicesList)() = (int (*)()) dlsym(commandLayer_handle, "Ethernet_RefresDevicesList");
    int (*MyGetEthernetConfig)(EthernetCommConfig) = (int (*)(EthernetCommConfig)) dlsym(commandLayer_handle, "Ethernet_GetEthernetConfiguration");

#elif _WIN32
    HINSTANCE commandLayer_handle = LoadLibrary(L"CommandLayerEthernet.dll");
    if (!commandLayer_handle) {
        cerr << "Error loading Kinova command layer library." << endl;
        return -1;
    }

    // Load functions from the library
    int (*MyInitAPI)() = (int (*)()) GetProcAddress(commandLayer_handle, "InitAPI");
    int (*MyCloseAPI)() = (int (*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) =
        (int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int&)) GetProcAddress(commandLayer_handle, "GetDevices");
    int (*MyRefresDevicesList)() = (int (*)()) GetProcAddress(commandLayer_handle, "RefresDevicesList");
#endif

    // Verify all functions are loaded
    if (!MyInitAPI || !MyCloseAPI || !MyGetDevices || !MyRefresDevicesList) {
        cerr << "Error resolving one or more functions from library." << endl;
#ifdef __linux__
        dlclose(commandLayer_handle);
#elif _WIN32
        FreeLibrary(commandLayer_handle);
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

    MyGetEthernetConfig getEthernetCommConfig;
    getEthernetCommConfig.localBcastPort = 25015;
    getEthernetCommConfig.localCmdport = 25025;
    getEthernetCommConfig.robotPort = 55000;
    getEthernetCommConfig.localIpAddress = inet_addr("192.168.100.X"); // Your custom IP
    getEthernetCommConfig.robotIpAddress = inet_addr("192.168.100.10");
    getEthernetCommConfig.subnetMask = inet_addr("255.255.255.0");
    getEthernetCommConfig.rxTimeOutInMs = 1000;

    // Refresh device list
    cout << "Refreshing devices list..." << endl;
    MyRefresDevicesList();

    // Get devices and check if any are available
    KinovaDevice devices[MAX_KINOVA_DEVICE];
    int deviceResult;
    int devicesCount = MyGetDevices(devices, deviceResult);

    if (devicesCount > 0) {
        cout << "Robot connected! Debug message: Robot with Serial Number " 
             << devices[0].SerialNumber << " is online." << endl;
    } else {
        cout << "No robot found on the network. Devices count: " << devicesCount << endl;
    }

    // Close API
    cout << "Closing API..." << endl;
    MyCloseAPI();

#ifdef __linux__ 
    dlclose(commandLayer_handle);
#elif _WIN32
    FreeLibrary(commandLayer_handle);
#endif

    return 0;
}

