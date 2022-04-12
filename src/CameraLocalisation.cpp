#include <CameraLocalisation.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace camera_localisation {

/* onInit() method //{ */
    void CameraLocalisation::onInit() {

        // | ---------------- set my booleans to false ---------------- |

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

        // | ---------------- subscribers initialize ------------------ |

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("CameraLocalisation");
        m_transformer.setDefaultPrefix(m_uav_name);

        // | -------------------- initialize timers ------------------- |
        m_tim_example = nh.createTimer(ros::Duration(1), &CameraLocalisation::m_tim_callb_example, this);
        ROS_INFO_ONCE("[CameraLocalisation]: initialized");

        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |

// | --------------------- timer callbacks -------------------- |

    void CameraLocalisation::m_tim_callb_example([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_is_initialized) {
            ROS_ERROR_ONCE("[CameraLocalisation]: timer: not initialised");
            return;
        }

        ROS_INFO_THROTTLE(1.0, "[CameraLocalisation]:timer callback published ");
    }


// | -------------------- other functions ------------------- |

}  // namespace camera_localisation  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(camera_localisation::CameraLocalisation, nodelet::Nodelet)
