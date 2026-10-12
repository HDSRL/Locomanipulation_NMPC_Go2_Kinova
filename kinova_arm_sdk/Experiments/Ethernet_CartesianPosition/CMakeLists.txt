cmake_minimum_required(VERSION 3.5)

# Project Declaration
project(RobotConnection)

# Specify the C++ Standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../API/)

############################################################
# Find and Link Threads (pthread)
############################################################

# Find the Threads package, which includes pthread on Linux
find_package(Threads REQUIRED)

############################################################
# Create Libraries
############################################################

# Import the Ethernet command and communication libraries
add_library(Kinova.API.EthCommandLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova.API.EthCommandLayerUbuntu PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../API/Kinova.API.EthCommandLayerUbuntu.so
)

add_library(Kinova.API.EthCommLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova.API.EthCommLayerUbuntu PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../API/Kinova.API.EthCommLayerUbuntu.so
)

add_library(Kinova.API.CommLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova.API.CommLayerUbuntu PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../API/Kinova.API.CommLayerUbuntu.so
)

############################################################
# Create Executable
############################################################

# Add an executable for the EthernetCartesian.cpp file
add_executable(RobotConnection EthernetCartesian.cpp)

# Link the imported libraries, system library `dl`, and Threads
target_link_libraries(RobotConnection
    PUBLIC 
        Kinova.API.EthCommandLayerUbuntu
        Kinova.API.EthCommLayerUbuntu
        Kinova.API.CommLayerUbuntu
        dl
        Threads::Threads  # Link against the Threads package (pthread)
)

