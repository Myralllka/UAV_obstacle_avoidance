cmake_minimum_required(VERSION 3.1.2)
project(uav_camera_localisation)

set(FILESNAME CameraLocalisation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -Werror -Werror=vla)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        mrs_lib
        nav_msgs
        cv_bridge
        apriltag_ros
        basler_stereopair_driver
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp mrs_lib cv_bridge
)

catkin_install_python(PROGRAMS scripts/epipolar_line.py scripts/publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        )

add_library(${FILESNAME} src/${FILESNAME}.cpp)

add_dependencies(${FILESNAME} ${${FILESNAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES})
target_link_libraries(${FILESNAME} ${OpenCV_LIBRARIES})
