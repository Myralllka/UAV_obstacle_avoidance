cmake_minimum_required(VERSION 3.1.2)
project(uav_camera_localization)

set(FILESNAME CameraLocalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -Werror -Werror=vla)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        rospy
        std_msgs
        mrs_lib
        mrs_msgs
        nav_msgs
        pcl_ros
        pcl_conversions
        tf2
        tf2_ros
        tf2_msgs
        tf2_geometry_msgs
        tf2_sensor_msgs
        apriltag_ros
        geometry_msgs
        image_geometry
        basler_stereopair_driver
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp mrs_lib geometry_msgs mrs_msgs pcl_ros nav_msgs
)

catkin_install_python(PROGRAMS scripts/epipolar_line.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        )
set(CPU_BASELINE SSE4_2)
set(CPU_BASELINE AVX2)
set(CMAKE_BUILD_TYPE Release)
add_library(${FILESNAME} src/${FILESNAME}.cpp src/Utils.cpp include/Utils.h)

add_dependencies(${FILESNAME} ${${FILESNAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES})
target_link_libraries(${FILESNAME} ${OpenCV_LIBRARIES})
target_link_libraries (${FILESNAME} ${Eigen_LIBRARIES})
