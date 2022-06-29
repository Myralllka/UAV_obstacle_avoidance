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
#include <random>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/geometry/conversions.h>

/* other important includes */
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

#include "Utils.h"
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
        bool m_is_initialized = false;
        bool m_debug_images;
        bool m_debug_matches;
        bool m_debug_distances;
        bool m_debug_markers;

        /* ros parameters */
        std::string m_uav_name;

        /* other parameters */
        std::string m_name_base;
        std::string m_name_CL;
        std::string m_name_CR;
        std::string m_method_triang;

        // | --------------------- Opencv transformer -------------------- |
        geometry_msgs::TransformStamped m_RL_transform, m_LR_transform;
        cv::Mat m_P_L_cv, m_P_R_cv;
        Eigen::Matrix<double, 3, 4> m_P_R_eig, m_P_L_eig;

        cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
        cv::Ptr<cv::Feature2D> detector;
        Eigen::Matrix<double, 3, 3> m_K_CL_eig, m_K_CR_eig;
        cv::Mat m_K_CL_cv, m_K_CR_cv;
        float m_distance_ratio;
        size_t m_distance_threshold;
        // TODO: generalize
        cv::Rect rect_l, rect_r;
        cv::Mat m_mask_left{cv::Mat::zeros(cv::Size{1600, 1200}, CV_8U)};
        cv::Mat m_mask_right{cv::Mat::zeros(cv::Size{1600, 1200}, CV_8U)};

        Eigen::Affine3d m_fleft_pose, m_fright_pose;
        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |

        // | --------------------- timer callbacks -------------------- |

        ros::Timer m_tim_corresp;

        [[maybe_unused]] void m_tim_cbk_corresp([[maybe_unused]] const ros::TimerEvent &ev);

        // | ----------------------- publishers ----------------------- |

        ros::Publisher m_pub_im_corresp;
        ros::Publisher m_pub_markarray;
        ros::Publisher m_pub_im_left_debug;
        ros::Publisher m_pub_im_right_debug;
        ros::Publisher m_pub_pcld;

        // | ----------------------- subscribers ---------------------- |
        mrs_lib::SubscribeHandler<sensor_msgs::Image> m_handler_imleft;
        mrs_lib::SubscribeHandler<sensor_msgs::Image> m_handler_imright;

        // | ---------------- pinhole camera models ------------------- |
        image_geometry::PinholeCameraModel m_camera_left;
        image_geometry::PinholeCameraModel m_camera_right;

        cv::Point3d OL_frameR, OR_frameL, m_o1_3d, m_o2_3d;
        cv::Point2d m_o1_2d, m_o2_2d;
        // | --------------------- other functions -------------------- |

        // ------------------------ UTILS -----------------------------

        void setUp();

        [[maybe_unused]] std::vector<Eigen::Vector3d> triangulate_primitive(const std::vector<cv::Point2d> &kpts1,
                                                                            const std::vector<cv::Point2d> &kpts2);

        [[maybe_unused]] void m_detect_and_compute_kpts(const cv::Mat &img,
                                                        const cv::Mat &mask,
                                                        std::vector<cv::KeyPoint> &res_kpts,
                                                        cv::Mat &res_desk);

        [[maybe_unused]] void filter_matches(const std::vector<cv::DMatch> &input_matches,
                                             const std::vector<cv::KeyPoint> &kpts1,
                                             const std::vector<cv::KeyPoint> &kpts2,
                                             const cv::Point2d &o1_2d,
                                             const cv::Point2d &o2_2d,
                                             std::vector<cv::DMatch> &res_matches,
                                             std::vector<cv::Point2d> &res_kpts1,
                                             std::vector<cv::Point2d> &res_kpts2);


    };
//}

}  // namespace camera_localization
