#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

/* other important includes */
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>

/* opencv */
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
//}

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
        std::string m_fleft_topic_name;
        std::string m_fright_topic_name;

        /* other parameters */

        cv::Rect m_roi;


        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        [[maybe_unused]] void m_callb_example([[maybe_unused]] const nav_msgs::Odometry::ConstPtr &msg);

        [[maybe_unused]] void m_callb_crop_image([[maybe_unused]] const sensor_msgs::ImageConstPtr &msg);

        // | --------------------- timer callbacks -------------------- |
        ros::Timer m_tim_example;

        [[maybe_unused]] void m_tim_callb_example([[maybe_unused]] const ros::TimerEvent &ev);

        // | ----------------------- publishers ----------------------- |
        ros::Publisher m_pub_fright_roi;
        ros::Publisher m_pub_fleft_roi;

        // | ----------------------- subscribers ---------------------- |
        ros::Subscriber m_sub_fright_rect;
        ros::Subscriber m_sub_fleft_rect;
        // | --------------------- other functions -------------------- |

    };
//}

}  // namespace camera_localisation
