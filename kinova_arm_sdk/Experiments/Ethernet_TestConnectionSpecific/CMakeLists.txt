cmake_minimum_required(VERSION 3.5)

project(RobotConnection)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../API/)

############################################################
# Create libraries
############################################################

# Import the Ethernet command and communication libraries
add_library(Kinova.API.EthCommandLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova.API.EthCommandLayerUbuntu PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../API/Kinova.API.EthCommandLayerUbuntu.so)

add_library(Kinova.API.EthCommLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova.API.EthCommLayerUbuntu PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../API/Kinova.API.EthCommLayerUbuntu.so)

############################################################
# Create an executable
############################################################

# Add an executable for the main.cpp file
add_executable(RobotConnection EthernetTest.cpp)

# Link the imported libraries and system library `dl` (for Linux)
target_link_libraries(RobotConnection
    PUBLIC 
        Kinova.API.EthCommandLayerUbuntu
        Kinova.API.EthCommLayerUbuntu
        dl
)

