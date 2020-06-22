cmake_minimum_required(VERSION 3.0.0)

add_compile_options( -lcurlpp -lcurl -D BUILD_TIFF=ON)

project(quadrupedrobot)

set(GAZEBOMSG_SRC_DIR source/gazebo/message)
set(GAZEBOSENSOR_SRC_DIR source/gazebo/sensor)
set(MATLABMSG_SRC_DIR source/matlab/message)
set(DS_SRC_DIR source/ds)
set(ROBOT_SRC source/main.cpp  source/quadrupedrobot.cpp source/limbModel.cpp source/mpc.cpp source/gait.cpp source/gnuplot.cpp
        ${GAZEBOSENSOR_SRC_DIR}/sensor.cpp
        ${GAZEBOMSG_SRC_DIR}/jointsensors.pb.cc
        ${GAZEBOMSG_SRC_DIR}/contactsensors.pb.cc
        ${GAZEBOMSG_SRC_DIR}/jointsposition.pb.cc
        ${GAZEBOMSG_SRC_DIR}/control.pb.cc
        ${MATLABMSG_SRC_DIR}/matlabidentification.pb.cc
        ${DS_SRC_DIR}/traprocgen.cpp)

#Add Matlab support
#set(INC_DIR_CX  "/usr/local/MATLAB/R2018a/extern/include/")
#set(LINK_DIR_CX "/usr/local/MATLAB/R2018a/bin/glnxa64" "/usr/local/MATLAB/R2018a/sys/os/glnxa64")
#include_directories(${INC_DIR_CX})
#link_directories(${LINK_DIR_CX})

set(INC_DIR_CX  "."  "/home/cx/3.2/include")
set(LINK_DIR "/home/cx/3.2/libs")
include_directories(${INC_DIR_CX})
link_directories(${LINK_DIR})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Eigen support
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(quadrupedrobot ${ROBOT_SRC})
target_link_libraries(quadrupedrobot ${GAZEBO_LIBRARIES} qpOASES )

add_library(qrINSmessage_plugin SHARED "source/gazebo/qrINSmessage_plugin/qrINSmessage.cpp")
target_link_libraries(qrINSmessage_plugin ${GAZEBO_LIBRARIES})

add_library(qrmodel_plugin SHARED source/gazebo/qrmodel_plugin/qrmodelplugin.cpp
                        ${GAZEBOMSG_SRC_DIR}/jointsensors.pb.cc
                        ${GAZEBOMSG_SRC_DIR}/contactsensors.pb.cc
                        ${GAZEBOMSG_SRC_DIR}/jointsposition.pb.cc
                        ${GAZEBOMSG_SRC_DIR}/control.pb.cc)
target_link_libraries(qrmodel_plugin ${GAZEBO_LIBRARIES})


#add_library(test_imu_plugin  SHARED test_imu.cpp test_imu.h)
#target_link_libraries(test_imu_plugin ${GAZEBO_LIBRARIES})
