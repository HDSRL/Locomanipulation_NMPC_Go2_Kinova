cmake_minimum_required(VERSION 3.5)

# Project Declaration
project(RobotConnection)

# Specify the C++ Standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
# Use target_include_directories for better encapsulation
# Adjust the path to your API headers as necessary

############################################################
# Find and Link Threads (pthread)
############################################################

# Find the Threads package, which includes pthread on Linux
find_package(Threads REQUIRED)

############################################################
# Create Libraries
############################################################

# Import the Kinova API shared libraries
add_library(Kinova_API_EthCommandLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova_API_EthCommandLayerUbuntu PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../API/Kinova.API.EthCommandLayerUbuntu.so
)

add_library(Kinova_API_EthCommLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova_API_EthCommLayerUbuntu PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../API/Kinova.API.EthCommLayerUbuntu.so
)

add_library(Kinova_API_CommLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova_API_CommLayerUbuntu PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../API/Kinova.API.CommLayerUbuntu.so
)

# Optionally, ensure that the USBCommandLayer is available at runtime
# Since it's loaded via dlopen, it's not linked here but should be in the library path

############################################################
# Create Executable
############################################################

# Add an executable for your source file
# Ensure that the source file name matches your actual source
add_executable(RobotConnection EthernetOptimalGravity.cpp)


target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../API/)


# Link the imported libraries, system library `dl`, and Threads
target_link_libraries(RobotConnection
    PRIVATE 
        Kinova_API_EthCommandLayerUbuntu
        Kinova_API_EthCommLayerUbuntu
        Kinova_API_CommLayerUbuntu
        dl
        Threads::Threads  # Link against the Threads package (pthread)
)

############################################################
# Runtime Library Path Configuration
############################################################

# Set RPATH to find Kinova API shared libraries at runtime
# This avoids the need to set LD_LIBRARY_PATH manually
set_target_properties(RobotConnection PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../API/"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

############################################################
# Optional: Install Target
############################################################

# If you plan to install the executable, define installation rules
install(TARGETS RobotConnection
    RUNTIME DESTINATION bin
)
