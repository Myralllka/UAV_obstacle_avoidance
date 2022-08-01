#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <random>
#include <utility>
#include <chrono>
#include <thread>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/geometry/conversions.h>

/* other important includes */
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_geometry/pinhole_camera_model.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>

/* opencv */
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/ocl.hpp>

#include "Utils.h"
#include "barrier.hpp"
//}
namespace camera_localization {

/* class Cameralocalization //{ */
    class CameraLocalization : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        void onInit() override;

    private:

        const std::string NODENAME{"CameraLocalization"};
        /* flags */
        std::atomic<bool> m_is_initialized = false;

        bool m_debug_matches, m_debug_distances, m_debug_markers;

        bool m_img_ready = false, m_features_ready = false;

        /* ros parameters */
        std::string m_uav_name;

        /* other parameters */
        std::string m_name_base;
        std::string m_name_CL;
        std::string m_name_CR;
        std::string m_method_triang;
        std::string m_method_detect;
        std::string m_imgs_encoding;

        // | --------------------- Opencv -------------------- |
        geometry_msgs::TransformStamped m_RL_transform, m_LR_transform;
        cv::Mat m_P_L_cv, m_P_R_cv;
        Eigen::Matrix<double, 3, 4> m_P_R_eig, m_P_L_eig;

        // | --------------------- Detectors -------------------- |
        cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
        int m_n_features;
        Eigen::Matrix<double, 3, 3> m_K_CL_eig, m_K_CR_eig;
        cv::Matx33d m_K_CL_cv, m_K_CR_cv;
        float m_distance_ratio;
        size_t m_distance_threshold;
        // TODO: generalize
        cv::Rect rect_l, rect_r;
        cv::UMat m_mask_left{cv::UMat::zeros(cv::Size{1600, 1200}, CV_8U)};
        cv::UMat m_mask_right{cv::UMat::zeros(cv::Size{1600, 1200}, CV_8U)};
        std::vector<cv::UMat> m_masks;
        Eigen::Affine3d m_fleft_pose, m_fright_pose;
        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        void m_cbk_save_images(const sensor_msgs::ImageConstPtr &msg_left,
                               const sensor_msgs::ImageConstPtr &msg_right);

        // | --------------------- timer callbacks -------------------- |

        // | ----------------------- publishers ----------------------- |

        ros::Publisher m_pub_im_corresp;
        ros::Publisher m_pub_markarray;
        ros::Publisher m_pub_im_left_debug;
        ros::Publisher m_pub_im_right_debug;
        ros::Publisher m_pub_pcld;

        // | ----------------------- subscribers ---------------------- |
        message_filters::Subscriber<sensor_msgs::Image> m_sub_camfleft;
        message_filters::Subscriber<sensor_msgs::Image> m_sub_camfright;

        using approx_time_sync_images_t = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
        boost::shared_ptr<message_filters::Synchronizer<approx_time_sync_images_t>> m_time_sync;

        // | ------------------- detection modules -------------------- |
        cv::Mat m_img_debug_fleft, m_img_debug_fright;
        std::mutex m_mut_imgs{}, m_mut_features{};
        std::condition_variable m_cv_image{}, m_cv_features{};

        cv::Mat m_img_left, m_img_right;

        std::vector<feature_t> m_features{};
        // | ---------------- pinhole camera models ------------------- |
        image_geometry::PinholeCameraModel m_camera_left;
        image_geometry::PinholeCameraModel m_camera_right;

        cv::Point3d OL_frameR, OR_frameL, m_o1_3d, m_o2_3d;
        cv::Point2d m_o1_2d, m_o2_2d;

        // | --------------------- other functions -------------------- |

        [[maybe_unused]] void m_features_detection();

        [[maybe_unused]] void m_corresp_matching();

        // ------------------------ UTILS -----------------------------

        void setUp();

        [[maybe_unused]] std::vector<Eigen::Vector3d> triangulate_primitive(const std::vector<cv::Point2d> &kpts1,
                                                                            const std::vector<cv::Point2d> &kpts2);

        [[maybe_unused]] void
        filter_matches(const std::vector<cv::DMatch> &input_matches,
                       const std::vector<cv::KeyPoint> &kpts1,
                       const std::vector<cv::KeyPoint> &kpts2,
                       const cv::Point2d &o1_2d, const cv::Point2d &o2_2d,
                       std::vector<cv::DMatch> &res_matches,
                       std::vector<cv::Point2d> &res_kpts1, std::vector<cv::Point2d> &res_kpts2,
                       std::vector<cv::Point2d> &res_kpts1_rect, std::vector<cv::Point2d> &res_kpts2_rect);

    };
//}

}  // namespace camera_localization
