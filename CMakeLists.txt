cmake_minimum_required(VERSION 3.5)
project(A1_Robot VERSION 1.1.1)
set(CMAKE_BUILD_TYPE RELEASE)
# set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_COMPILER "/usr/bin/gcc-11")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -std=c++17 -ggdb")

set(LL_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# # ===== Path to RaiSim and RaiSimOgre ===== #
# if($ENV{USER} STREQUAL "basit")
#     find_package(raisim REQUIRED HINTS "/home/basit/workspace/raisimLib/build/lib/cmake/raisim")
#     find_package(raisim REQUIRED HINTS "/home/basit/workspace/raisimLib/raisim/linux/lib/cmake/raisim")
#     find_package(raisimOgre REQUIRED HINTS "/home/basit/workspace/raisimOgre/build/cmake")
# else()
#     # find_package(raisim REQUIRED PATHS "/home/basit/workspace/raisimLib/cmake/raisim")
#     # find_package(raisimOgre CONFIG PATHS "/home/basit/workspace/raisimOgre/cmake")
# endif()

# ===== RaiSim and RaiSimOgre ===== #
find_package(raisim CONFIG HINTS "$ENV{LOCAL_INSTALL}/lib/cmake/raisim")
find_package(OGRE CONFIG REQUIRED PATHS "$ENV{LOCAL_INSTALL}/lib/OGRE/cmake")
find_package(raisimOgre CONFIG HINTS  "$ENV{LOCAL_INSTALL}/share/raisimOgre/cmake")


# find_package(ifopt REQUIRED HINTS "/home/trec/WorkRaj/raisim_legged/ifopt")
# set(CMAKE_PREFIX_PATH "/home/trec/WorkRaj/raisim_legged/casadi/cmake")
find_package(casadi REQUIRED HINTS "/usr/local/lib/cmake/casadi")
# find_package(casadi REQUIRED)
if (casadi_FOUND)
    message(STATUS "CasADi found!")
else()
    message(FATAL_ERROR "CasADi not found!")
endif()

# set casadi include directory
set(CASADI_INCLUDE_DIR /usr/local/include/casadi)

# set IPOPT include directory
set(IPOPT_INCLUDE_DIRS /usr/include/coin)




find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
if (Eigen3_FOUND)
    message("Eigen found")
    message("  Eigen location: ${Eigen3_DIR}")
    message("  Eigen Version: ${Eigen3_VERSION}")
endif ()
find_package(Boost REQUIRED COMPONENTS timer system thread)
find_library(LCM lcm HINTS /usr/local/lib)
find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

if (raisimOgre_FOUND)
    include_directories(PUBLIC ${LL_DIR}/raisimOgre_include)
    file(COPY ${LL_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
endif()

# Use pkg-config to find the required flags for hidapi
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)

# include header directories
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/kinova_arm_sdk/API
    SYSTEM ${IPOPT_INCLUDE_DIRS}
    SYSTEM ${CASADI_INCLUDE_DIR}
    SYSTEM ${SNOPT_INCLUDE_DIRS}
    ${CODE_GEN_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    home/trec/WorkRaj/raisim_legged/libsnopt7)

# library directories
link_directories(
    ${LIBRARY_DIRS}
    ${IPOPT_LIBRARY_DIRS}
    home/trec/WorkRaj/raisim_legged/libsnopt7)

# find all the header files
file(GLOB HEADER_FILES_HPP ${CMAKE_SOURCE_DIR}/include/*.hpp)
file(GLOB HEADER_FILES_H ${CMAKE_SOURCE_DIR}/include/*.h)


set(UT_INC ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/include)
set(UT_LINK ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/lib)
set(EXTRA_LIBS -pthread ${UT_LINK}/libunitree_legged_sdk_amd64.so)


## Use GNUInstallDirs to install libraries into correct locations on all platforms.
include(GNUInstallDirs)


set(UT2_INC ${CMAKE_CURRENT_SOURCE_DIR}/unitree_sdk2/include)
set(UT2_LINK ${CMAKE_CURRENT_SOURCE_DIR}/unitree_sdk2/lib/x86_64)


set(DDS_INC ${CMAKE_CURRENT_SOURCE_DIR}/unitree_sdk2/thirdparty/include/ddscxx)
set(DDS_LIB ${CMAKE_CURRENT_SOURCE_DIR}/unitree_sdk2/thirdparty/lib/x86_64)

set(DDSH_INC ${CMAKE_CURRENT_SOURCE_DIR}/unitree_sdk2/thirdparty/include)

set(ROS_INC )

set(EXTRA_LIBS_2 -pthread ${UT2_LINK}/libunitree_sdk2.a 
    ${DDS_LIB}/libddscxx.so
    ${DDS_LIB}/libddsc.so)
# ## Import thirdparty libraries
# add_subdirectory(unitree_sdk2/thirdparty)


# find casadi library
find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

# Find the Threads package, which includes pthread on Linux
find_package(Threads REQUIRED)


# ===== Dynamics Library ===== #

# Settings:
set(COMPILE_CORIOLIS FALSE) # Option to compile the coriolis terms. Not implemented in code though.

set(dyn_srcs 
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_D_B_G.cpp"
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_FK.cpp"
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_FK.cpp"
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_Jaco.cpp"
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_Jaco_dot.cpp")
if (COMPILE_CORIOLIS)
    set(dyn_srcs ${dyn_srcs} "${LL_DIR}/dynamics/A1_New_wFoot/A1_C.cpp")
endif()
add_library(dyn_lib ${dyn_srcs})
target_include_directories(dyn_lib PUBLIC
    "${LL_DIR}/dynamics"
    "${LL_DIR}/dynamics/A1_New_wFoot"
)

# ===== Controller Library ===== #
add_library(ctrl_lib
    "${LL_DIR}/src/Parameters.cpp"
    "${LL_DIR}/src/RobotModel.cpp"
    "${LL_DIR}/src/LocoWrapper.cpp"
    "${LL_DIR}/src/LowLevelCtrl.cpp"
    "${LL_DIR}/src/VirtualConstraints.cpp"
    "${LL_DIR}/src/ContactEst.cpp"
    "${LL_DIR}/src/MotionPlanner.cpp"
    "${LL_DIR}/src/DataLog.cpp"
    "${LL_DIR}/src/MPC_dist.cpp"
    "${LL_DIR}/src/nmpc.cpp"
    "${LL_DIR}/src/KinovaArm.cpp"
    "${LL_DIR}/src/JoyStick.cpp"
)

add_library(ctrl_lib_a1_kinova
    "${LL_DIR}/src/Parameters.cpp"
    "${LL_DIR}/src/RobotModel.cpp"
    "${LL_DIR}/src/LocoWrapper.cpp"
    "${LL_DIR}/src/LowLevelCtrl.cpp"
    "${LL_DIR}/src/VirtualConstraints.cpp"
    "${LL_DIR}/src/ContactEst.cpp"
    "${LL_DIR}/src/MotionPlanner.cpp"
    "${LL_DIR}/src/DataLog.cpp"
    "${LL_DIR}/src/MPC_dist.cpp"
    "${LL_DIR}/src/a1_kinova_nmpc.cpp"
    "${LL_DIR}/src/KinovaArm.cpp"
    "${LL_DIR}/src/JoyStick.cpp"

)

target_include_directories(ctrl_lib PUBLIC 
    ${LL_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${LL_DIR}/optimization/iSWIFT/include
    ${LL_DIR}/optimization/iSWIFT/ldl/include
    ${LL_DIR}/optimization/iSWIFT/cpp_wrapper
)
target_link_libraries(ctrl_lib PRIVATE dyn_lib)

target_include_directories(ctrl_lib_a1_kinova PUBLIC 
    ${LL_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${LL_DIR}/optimization/iSWIFT/include
    ${LL_DIR}/optimization/iSWIFT/ldl/include
    ${LL_DIR}/optimization/iSWIFT/cpp_wrapper
)
target_link_libraries(ctrl_lib_a1_kinova PRIVATE dyn_lib)

# ===== High Level Library (basic test only) ===== #
# add_library(hl_lib
# 	"${LL_DIR}/src/forceUpdate.cpp"
# )

# ===== QP SWIFT Library ===== #
add_library(optim_lib
    "${LL_DIR}/optimization/iSWIFT/src/Auxilary.c"
    "${LL_DIR}/optimization/iSWIFT/src/Prime.c"
    "${LL_DIR}/optimization/iSWIFT/src/timer.c"
    "${LL_DIR}/optimization/iSWIFT/ldl/src/ldl.c"
    "${LL_DIR}/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp"
)
target_include_directories(optim_lib PUBLIC 
    ${EIGEN3_INCLUDE_DIRS}
    ${LL_DIR}/optimization/iSWIFT/include
    ${LL_DIR}/optimization/iSWIFT/ldl/include
    ${LL_DIR}/optimization/iSWIFT/cpp_wrapper
)


# ===== Kinova Library ===== #
# Import the Kinova API shared libraries
add_library(Kinova_API_EthCommandLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova_API_EthCommandLayerUbuntu PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kinova_arm_sdk/API/Kinova.API.EthCommandLayerUbuntu.so
)

add_library(Kinova_API_EthCommLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova_API_EthCommLayerUbuntu PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kinova_arm_sdk/API/Kinova.API.EthCommLayerUbuntu.so
)

add_library(Kinova_API_CommLayerUbuntu SHARED IMPORTED)
set_target_properties(Kinova_API_CommLayerUbuntu PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kinova_arm_sdk/API/Kinova.API.CommLayerUbuntu.so
)
# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/kinova_arm_sdk/API/)

# ==== JoyShockLibrary ==== #
add_library(JoyShockLibrary STATIC IMPORTED)
set_target_properties(JoyShockLibrary PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/JoyShockLibrary/build/JoyShockLibrary/libJoyShockLibrary.a
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/JoyShockLibrary/JoyShockLibrary)



# ===== Executables ===== #
if (raisimOgre_FOUND)
    add_compile_definitions(RUNNING_SIM) # adds the toe offset to be used in simulation (exp needs no offset)
    add_executable(Run_Sim src/A1_Sim.cpp )
    # target_link_libraries(Run_Sim PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ifopt::ifopt_snopt ipopt -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_link_libraries(Run_Sim PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ipopt -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_compile_definitions(Run_Sim PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")

    add_compile_definitions(RUNNING_SIM) # adds the toe offset to be used in simulation (exp needs no offset)
    add_executable(Run_Sim_Debug src/A1_Sim_debug.cpp )
    # target_link_libraries(Run_Sim PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ifopt::ifopt_snopt ipopt -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_link_libraries(Run_Sim_Debug PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ipopt ${EXTRA_LIBS} ${EXTRA_LIBS_2} ${LCM} ${Boost_LIBRARIES} -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_include_directories(Run_Sim_Debug PRIVATE ${UT_INC} ${UT2_INC} ${DDS_INC} ${DDSH_INC})
    target_compile_definitions(Run_Sim_Debug PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")

    add_compile_definitions(RUNNING_SIM) # adds the toe offset to be used in simulation (exp needs no offset)
    add_executable(A1_Kinova_Sim_Debug src/A1_Kinova_Sim_debug.cpp )
    # target_link_libraries(Run_Sim PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ifopt::ifopt_snopt ipopt -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_link_libraries(A1_Kinova_Sim_Debug PRIVATE ctrl_lib_a1_kinova optim_lib dyn_lib raisim::raisimOgre ipopt ${EXTRA_LIBS} ${EXTRA_LIBS_2} ${LCM} ${Boost_LIBRARIES} -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp JoyShockLibrary ${HIDAPI_LIBRARIES})
    target_include_directories(A1_Kinova_Sim_Debug PRIVATE ${UT_INC} ${UT2_INC} ${DDS_INC} ${DDSH_INC})
    target_compile_definitions(A1_Kinova_Sim_Debug PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")
    

    add_compile_definitions(RUNNING_SIM) # adds the toe offset to be used in simulation (exp needs no offset)
    add_executable(A1_Kinova_Run_Sim src/A1_Kinova_Sim.cpp )
    # target_link_libraries(Run_Sim PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ifopt::ifopt_snopt ipopt -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_link_libraries(A1_Kinova_Run_Sim PRIVATE ctrl_lib_a1_kinova optim_lib dyn_lib raisim::raisimOgre ipopt -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_compile_definitions(A1_Kinova_Run_Sim PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")
# else()
    add_executable(A1_Experiment_w_HL src/A1_Exp_LL_w_NMPC.cpp )
    target_link_libraries(A1_Experiment_w_HL PRIVATE ctrl_lib optim_lib dyn_lib ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES} ipopt -Llibsnopt7_cpp -Llibassimp ${CASADI_LIBRARIES} yaml-cpp)
    target_include_directories(A1_Experiment_w_HL PRIVATE ${UT_INC})
    
    add_executable(Run_Exp_Debug src/A1_Exp_debug.cpp )
    target_link_libraries(Run_Exp_Debug PRIVATE ctrl_lib optim_lib dyn_lib ${EXTRA_LIBS} ${EXTRA_LIBS_2} ${LCM} ${Boost_LIBRARIES} ipopt -Llibsnopt7_cpp -Llibassimp ${CASADI_LIBRARIES} yaml-cpp)
    target_include_directories(Run_Exp_Debug PRIVATE ${UT2_INC} ${UT_INC} ${DDS_INC} ${DDSH_INC})
    
    add_executable(Run_Exp_Sep_Kinova src/A1_Exp_Sep_Kinova.cpp )
    target_link_libraries(Run_Exp_Sep_Kinova PRIVATE ctrl_lib optim_lib dyn_lib ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES} ipopt -Llibsnopt7_cpp -Llibassimp ${CASADI_LIBRARIES} yaml-cpp Kinova_API_EthCommandLayerUbuntu Kinova_API_EthCommLayerUbuntu Kinova_API_CommLayerUbuntu dl Threads::Threads)
    target_include_directories(Run_Exp_Sep_Kinova PRIVATE ${UT_INC} ${CMAKE_CURRENT_SOURCE_DIR}/kinova_arm_sdk/API/)
    
    add_executable(Run_A1_Kinova_Exp_Debug src/A1_Kinova_Exp_debug.cpp )
    target_link_libraries(Run_A1_Kinova_Exp_Debug PRIVATE ctrl_lib_a1_kinova optim_lib dyn_lib ${EXTRA_LIBS} ${EXTRA_LIBS_2} ${LCM} ${Boost_LIBRARIES} ipopt -Llibsnopt7_cpp -Llibassimp ${CASADI_LIBRARIES} yaml-cpp Kinova_API_EthCommandLayerUbuntu Kinova_API_EthCommLayerUbuntu Kinova_API_CommLayerUbuntu dl Threads::Threads JoyShockLibrary ${HIDAPI_LIBRARIES})
    target_include_directories(Run_A1_Kinova_Exp_Debug PRIVATE ${UT2_INC} ${UT_INC} ${DDS_INC} ${DDSH_INC} ${CMAKE_CURRENT_SOURCE_DIR}/kinova_arm_sdk/API/)


    add_compile_definitions(RUNNING_SIM) # adds the toe offset to be used in simulation (exp needs no offset)
    add_executable(Run_Sim_threaded src/A1_Sim_threaded.cpp )
    # target_link_libraries(Run_Sim PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ifopt::ifopt_snopt ipopt -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_link_libraries(Run_Sim_threaded PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ipopt ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES} -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_include_directories(Run_Sim_threaded PRIVATE ${UT_INC})
    target_compile_definitions(Run_Sim_threaded PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")

    
    add_compile_definitions(RUNNING_SIM) # adds the toe offset to be used in simulation (exp needs no offset)
    add_executable(Run_Sim_Kinova_threaded src/A1_Sim_Kinova_threaded.cpp )
    # target_link_libraries(Run_Sim PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ifopt::ifopt_snopt ipopt -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_link_libraries(Run_Sim_Kinova_threaded PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ipopt ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES} -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp Kinova_API_EthCommandLayerUbuntu Kinova_API_EthCommLayerUbuntu Kinova_API_CommLayerUbuntu dl Threads::Threads)  # Link against the Threads package (pthread))
    target_include_directories(Run_Sim_Kinova_threaded PRIVATE ${UT_INC} ${CMAKE_CURRENT_SOURCE_DIR}/kinova_arm_sdk/API/)
    target_compile_definitions(Run_Sim_Kinova_threaded PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")


    add_compile_definitions(RUNNING_SIM) # adds the toe offset to be used in simulation (exp needs no offset)
    add_executable(Run_A1_Kinova_Sim_Kinova_threaded src/A1_Kinova_Sim_Kinova_threaded.cpp )
    # target_link_libraries(Run_Sim PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ifopt::ifopt_snopt ipopt -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp)
    target_link_libraries(Run_A1_Kinova_Sim_Kinova_threaded PRIVATE ctrl_lib_a1_kinova optim_lib dyn_lib raisim::raisimOgre ipopt ${EXTRA_LIBS} ${EXTRA_LIBS_2} ${LCM} ${Boost_LIBRARIES} -Llibsnopt7_cpp ${CASADI_LIBRARIES} yaml-cpp Kinova_API_EthCommandLayerUbuntu Kinova_API_EthCommLayerUbuntu Kinova_API_CommLayerUbuntu dl Threads::Threads JoyShockLibrary ${HIDAPI_LIBRARIES})  # Link against the Threads package (pthread))
    target_include_directories(Run_A1_Kinova_Sim_Kinova_threaded PRIVATE ${UT2_INC} ${UT_INC} ${DDS_INC} ${DDSH_INC} ${CMAKE_CURRENT_SOURCE_DIR}/kinova_arm_sdk/API/)
    target_compile_definitions(Run_A1_Kinova_Sim_Kinova_threaded PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")
    ############################################################
    # Runtime Library Path Configuration
    ############################################################

    # # Set RPATH to find Kinova API shared libraries at runtime
    # # This avoids the need to set LD_LIBRARY_PATH manually
    # set_target_properties(Run_Sim_Kinova_threaded PROPERTIES
    # BUILD_WITH_INSTALL_RPATH TRUE
    # INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/kinova_arm_sdk/API/"
    # INSTALL_RPATH_USE_LINK_PATH TRUE
    # )

    # ############################################################
    # # Optional: Install Target
    # ############################################################

    # # If you plan to install the executable, define installation rules
    # install(TARGETS Run_Sim_Kinova_threaded
    # RUNTIME DESTINATION bin
    # )


    add_executable(restore src/restoreConfig.cpp)
    target_link_libraries(restore PRIVATE ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES})
    target_include_directories(restore PRIVATE ${UT_INC})

    add_executable(Joystick_test src/Joystick_test.cpp)
    target_link_libraries(Joystick_test PRIVATE JoyShockLibrary ${HIDAPI_LIBRARIES})

endif()





# # ===== GTTest ===== #

# # Enable testing
# enable_testing()

# # Manually specify GoogleTest paths
# set(GTEST_ROOT "/usr")
# set(GTEST_INCLUDE_DIR "/usr/include")
# set(GTEST_LIBRARY "/usr/lib/libgtest.a")
# set(GTEST_MAIN_LIBRARY "/usr/lib/libgtest_main.a")

# # Find GoogleTest
# find_package(GTest REQUIRED)


# # Ensure GTest is properly linked
# if(NOT TARGET GTest::GTest)
#     add_library(GTest::GTest UNKNOWN IMPORTED)
#     set_target_properties(GTest::GTest PROPERTIES
#         IMPORTED_LOCATION ${GTEST_LIBRARY}
#         INTERFACE_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIR})
# endif()

# if(NOT TARGET GTest::Main)
#     add_library(GTest::Main UNKNOWN IMPORTED)
#     set_target_properties(GTest::Main PROPERTIES
#         IMPORTED_LOCATION ${GTEST_MAIN_LIBRARY}
#         INTERFACE_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIR})
# endif()


# # Add test executable
# add_executable(moving_average_filter_test src/test/MovingAverageFilterTest.cpp)

# # Link GoogleTest manually
# target_include_directories(moving_average_filter_test PRIVATE ${GTEST_INCLUDE_DIR})
# target_link_libraries(moving_average_filter_test PRIVATE GTest::GTest GTest::Main pthread)

# # Register test with CTest
# add_test(NAME moving_average_filter_gtests COMMAND moving_average_filter_test)



# EOF