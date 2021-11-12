#include <CameraLocalisation.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
#include "CameraLocalisation.h"
namespace camera_localisation {

/* onInit() method //{ */
    void CameraLocalisation::onInit() {

        // | ---------------- set my booleans to false ---------------- |
        // but remember, always set them to their default value in the header file
        // because, when you add new one later, you might forget to come back here

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */
        mrs_lib::ParamLoader pl(nh, "CameraLocalisation");

        pl.loadParam("UAV_NAME", m_uav_name);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[CameraLocalisation]: failed to load non-optional parameters!");
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[CameraLocalisation]: loaded parameters");
        }
        // | ---------------- some data post-processing --------------- |

        // | ----------------- publishers initialize ------------------ |
        m_pub_example = nh.advertise<nav_msgs::Odometry>("odometry_echo", 1); // last param for queue size

        // | ---------------- subscribers initialize ------------------ |
        m_sub_example = nh.subscribe("/odometry/odom_main", 1, &CameraLocalisation::m_callb_example, this); // second parameter for queue size

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("CameraLocalisation", m_uav_name);

        // | -------------------- initialize timers ------------------- |
        m_tim_example = nh.createTimer(ros::Duration(0.1), &CameraLocalisation::m_tim_callb_example, this);
        ROS_INFO_ONCE("[CameraLocalisation]: initialized");

        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |
    [[maybe_unused]] void CameraLocalisation::m_callb_example([[maybe_unused]] const nav_msgs::Odometry::ConstPtr &msg) {
        if (not m_is_initialized) return;
    }

// | --------------------- timer callbacks -------------------- |
    void CameraLocalisation::m_tim_callb_example([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_is_initialized) return;
    }
// | -------------------- other functions ------------------- |

}  // namespace camera_localisation  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(camera_localisation::CameraLocalisation, nodelet::Nodelet)
