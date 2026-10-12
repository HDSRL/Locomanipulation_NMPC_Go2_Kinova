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
#include <arpa/inet.h>
#elif _WIN32
#include <Windows.h>
#include "CommandLayer.h"
#include "CommunicationLayer.h"
#endif

using namespace std;



int main()
{
    void* commandLayer_handle = dlopen("Kinova.API.EthCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
    void* commLayer_handle = dlopen("Kinova.API.EthCommLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);

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
    int (*MySetEthernetConfiguration)(EthernetConfiguration* config) = (int (*)(EthernetConfiguration * config)) dlsym(commandLayer_handle, "Ethernet_SetEthernetConfiguration");
    int (*MyGetEthernetConfiguration)(EthernetConfiguration* config) = (int (*)(EthernetConfiguration * config)) dlsym(commandLayer_handle, "Ethernet_GetEthernetConfiguration");
    int (*MyEthernet_InitEthernetAPI)(EthernetCommConfig* config) = (int (*)(EthernetCommConfig * config)) dlsym(commandLayer_handle, "Ethernet_InitEthernetAPI");
    
    
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
    

    EthernetCommConfig gEthernetCommConfig;
    gEthernetCommConfig.localBcastPort = 25015;
    gEthernetCommConfig.localCmdport = 25025;
    gEthernetCommConfig.robotPort = 55000;
    gEthernetCommConfig.localIpAddress = inet_addr("192.168.123.101"); // Your custom IP
    gEthernetCommConfig.robotIpAddress = inet_addr("192.168.123.19");
    gEthernetCommConfig.subnetMask = inet_addr("255.255.255.0");
    gEthernetCommConfig.rxTimeOutInMs = 1000;
    
	//printf("LocalIp: 0x%lx\n", config.robotIpAddress);
	//printf("LocalIp: %llu\n", config.robotIpAddress);
    //cout << config.localIpAddress << " " << config.robotIpAddress << " " << config.localCmdport << " " << config.localBcastPort << " " << config.robotPort << " " << config.rxTimeOutInMs << endl;

    // int initResult = MyInitCommunicationEthernet(&gEthernetCommConfig);
    



    int initCommandResult = MyEthernet_InitEthernetAPI(&gEthernetCommConfig);


    if (initCommandResult != 1) {
        cerr << "Failed to initialize the API. Return code: " << initCommandResult << endl;
        return -1;
    } else {
        cout << "API initialized successfully." << endl;
    }
    
    // MyGetEthernetConfiguration(&config_get);
    
    // std::cout << "Ip: " << (int)config_get.IPAddress[0] << "."<< (int)config_get.IPAddress[1] << "." << (int)config_get.IPAddress[2] << "." << (int)config_get.IPAddress[3]<<std::endl; 

    // Refresh device list
    cout << "Refreshing devices list..." << endl;
    MyRefresDevicesList();
    EthernetConfiguration config;
    MyGetEthernetConfiguration(&config);
    std::cout << "IP Address: " 
              << static_cast<int>(config.IPAddress[0]) << "."
              << static_cast<int>(config.IPAddress[1]) << "."
              << static_cast<int>(config.IPAddress[2]) << "."
              << static_cast<int>(config.IPAddress[3]) << std::endl;

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

