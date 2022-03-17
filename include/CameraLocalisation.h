#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/CameraInfo.h>
/* some STL includes */
#include <cstdlib>
#include <cstdio>
#include <vector>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

/* other important includes */
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/image_encodings.h>

/* opencv */
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/calib3d.hpp"

//}
template<typename T>
T deg2rad(const T x) { return x * M_PI / 180; }

template<typename T>
T rad2deg(const T x) { return x / M_PI * 180; }

namespace camera_localisation {

/* class CameraLocalisation //{ */
    class CameraLocalisation : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool m_is_initialized = false;

        /* ros parameters */
        std::string m_uav_name;

        /* other parameters */
        // | --------------------- MRS transformer -------------------- |
        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |

        // | --------------------- timer callbacks -------------------- |
        ros::Timer m_tim_example;

        [[maybe_unused]] void m_tim_callb_example([[maybe_unused]] const ros::TimerEvent &ev);

        // | ----------------------- publishers ----------------------- |

        // | ----------------------- subscribers ---------------------- |

        // | --------------------- other functions -------------------- |
    };
//}

}  // namespace camera_localisation
