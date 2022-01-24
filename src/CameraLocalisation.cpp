#include <CameraLocalisation.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

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

        int roi_start_x, roi_start_y, roi_h, roi_w;
        pl.loadParam("UAV_NAME", m_uav_name);
        pl.loadParam("fwd_left/rect_topic_name", m_fleft_topic_name);
        pl.loadParam("fwd_right/rect_topic_name", m_fright_topic_name);
        pl.loadParam("start_x", roi_start_x);
        pl.loadParam("start_y", roi_start_y);
        pl.loadParam("h", roi_h);
        pl.loadParam("w", roi_w);

        m_roi = cv::Rect{roi_start_x, roi_start_y, roi_w, roi_h};

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[CameraLocalisation]: failed to load non-optional parameters!");
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[CameraLocalisation]: loaded parameters");
        }
        // | ---------------- some data post-processing --------------- |

        // | ----------------- publishers initialize ------------------ |
        m_pub_fright_roi = nh.advertise<sensor_msgs::Image>("image_fwd_right_rect_cropped",
                                                            8); // last param for queue size
        m_pub_fleft_roi = nh.advertise<sensor_msgs::Image>("image_fwd_left_rect_cropped",
                                                           8);
        // | ---------------- subscribers initialize ------------------ |
        m_sub_fleft_rect = nh.subscribe(m_fleft_topic_name,
                                        8,
                                        &CameraLocalisation::m_callb_crop_image,
                                        this); // second parameter for queue size
        m_sub_fright_rect = nh.subscribe(m_fright_topic_name,
                                         8,
                                         &CameraLocalisation::m_callb_crop_image,
                                         this);

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("CameraLocalisation", m_uav_name);

        // | -------------------- initialize timers ------------------- |
        m_tim_example = nh.createTimer(ros::Duration(1), &CameraLocalisation::m_tim_callb_example, this);
        ROS_INFO_ONCE("[CameraLocalisation]: initialized");

        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |
    void CameraLocalisation::m_callb_crop_image(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR_THROTTLE(1.0, "[CameraLocalisation]: cv_bridge exception: %s", e.what());
            return;
        }

        cv_ptr->image = cv_ptr->image(m_roi);

        cv_ptr->header.frame_id = msg->header.frame_id + "_roi";

        auto n_r = cv_ptr->header.frame_id.find("right");
        auto n_l = cv_ptr->header.frame_id.find("left");

        if (n_r != std::string::npos) {
            m_pub_fright_roi.publish(cv_ptr->toImageMsg());
        } else if (n_l != std::string::npos) {
            m_pub_fleft_roi.publish(cv_ptr->toImageMsg());
        } else {
            ROS_ERROR_THROTTLE(1.0, "[CameraLocalisation]: wrong header frame id: %s", msg->header.frame_id.c_str());
            return;
        }
        //std::cout << "[D][CameraLocalisation]:" << cv_ptr->header.frame_id << std::endl;
        // chessboard detection
        cv::Size pattern_size{9, 7};
        cv::Size sqr_size{8, 8};
        cv::Size zero_zone_size{-1, -1};
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        //if (pattern_found) {
        //    std::ostringstream filename;
        //    filename << "image_" << std::to_string(counter++) << ".png";
        //    cv::imwrite(filename.str(), cv_ptr->image);
        //}
    }

// | --------------------- timer callbacks -------------------- |

    void CameraLocalisation::m_tim_callb_example([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_is_initialized) {
            ROS_ERROR_ONCE("[CameraLocalisation]: timer: not initialised");
            return;
        }

        std::cout << std::abs(static_cast<double>(left->header.stamp.toNSec()) -
                              static_cast<double>(right->header.stamp.toNSec())) << std::endl;

        ROS_INFO_THROTTLE(1.0, "[CameraLocalisation]:timer callback published ");
    }


// | -------------------- other functions ------------------- |

}  // namespace camera_localisation  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(camera_localisation::CameraLocalisation, nodelet::Nodelet)
