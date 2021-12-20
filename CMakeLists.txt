# CMakeLists.txt
project(imu_analysis)
cmake_minimum_required(VERSION 2.8)
## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories("/usr/include/eigen3"
            ${OpenCV_INCLUDE_DIRS}
            ${CERES_INCLUDE_DIRS})

set(ACC_LIB_SOURCE_FILES
            ${PROJECT_SOURCE_DIR}/acc_lib/allan_acc.cpp
            ${PROJECT_SOURCE_DIR}/acc_lib/fitallan_acc.cpp
            )
set(GYR_LIB_SOURCE_FILES
            ${PROJECT_SOURCE_DIR}/gyr_lib/allan_gyr.cpp
            ${PROJECT_SOURCE_DIR}/gyr_lib/fitallan_gyr.cpp
            )
        
add_executable(imu_an
            imu_an.cpp
            ${GYR_LIB_SOURCE_FILES}
            ${ACC_LIB_SOURCE_FILES}
            )
        
target_link_libraries(imu_an ${OpenCV_LIBS} ${CERES_LIBRARIES})