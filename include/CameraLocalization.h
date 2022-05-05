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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* opencv */
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

//}
template<typename T>
T deg2rad(const T x) { return x * M_PI / 180; }

template<typename T>
T rad2deg(const T x) { return x / M_PI * 180; }

template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> sqs(const Eigen::MatrixBase<Derived> &vec) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
}

Eigen::Matrix3d f2K33(const boost::array<double, 12> &P_in) {
    Eigen::Matrix3d K_out = Eigen::Matrix3d::Identity();
    K_out(0, 0) = P_in[0];
    K_out(0, 2) = P_in[2];
    K_out(1, 1) = P_in[5];
    K_out(1, 2) = P_in[6];
    return K_out;
}

inline Eigen::Vector3d cross(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    return {a.y() * b.z() - a.z() * b.y(),
            a.z() * b.x() - a.x() * b.z(),
            a.x() * b.y() - a.y() * b.x()};
}

[[maybe_unused]] std::pair<cv::Point2d, cv::Point2d> line2image(const cv::Point3d &line, int imwidth) {
    auto x0 = .0f;
    auto x1 = static_cast<double>(imwidth);
    double l0 = line.x;
    double l1 = line.y;
    double l2 = line.z;
    double y0 = -l2 / l1;
    double y1 = -(l2 + l0 * imwidth) / l1;

    return {cv::Point{static_cast<int>(std::round(x0)), static_cast<int>(std::ceil(y0))},
            cv::Point{static_cast<int>(std::round(x1)), static_cast<int>(std::ceil(y1))}};
}

[[maybe_unused]] inline void normalize_point(Eigen::Vector3d &p) {
    p.x() /= p.z();
    p.y() /= p.z();
    p.z() /= p.z();
}

[[maybe_unused]] inline void normalize_line(Eigen::Vector3d &p) {
    auto div = std::sqrt(std::pow(p.x(), 2) + std::pow(p.y(), 2));
    p.x() /= div;
    p.y() /= div;
    p.z() /= div;
}

cv::Point2d PX2u(const Eigen::Matrix<double, 3, 4> &P,
                 const Eigen::Vector3d &x) {
    Eigen::Matrix<double, 4, 1> homogenous_3d(x.x(), x.y(), x.z(), 1.0);
    Eigen::Vector3d homogenous_2d = P * Eigen::Vector4d{x.x(), x.y(), x.z(), 1};
    normalize_point(homogenous_2d);
    return {homogenous_2d.x(), homogenous_2d.y()};
}

std::vector<Eigen::Vector3d> X2td(const cv::Mat &input) {
    std::vector<Eigen::Vector3d> res;
    for (int i = 0; i < input.cols; ++i) {
        const auto x = input.at<double>(0, i);
        const auto y = input.at<double>(1, i);
        const auto z = input.at<double>(2, i);
        const auto w = input.at<double>(3, i);
        res.emplace_back(x / w, y / w, z / w);
    }
    return res;
}

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
        bool m_debug_matches;
        bool m_debug_epipolar;
        bool m_debug_markers;
        bool m_debug_projective_error;

        /* ros parameters */
        std::string m_uav_name;

        /* other parameters */
        std::string m_name_base;
        std::string m_name_CL;
        std::string m_name_CR;

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
        mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> m_handler_camleftinfo;
        mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> m_handler_camrightinfo;

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

        [[maybe_unused]] static std::vector<Eigen::Vector3d> triangulate_tdv(const Eigen::Matrix<double, 3, 4> &P1,
                                                                             const Eigen::Matrix<double, 3, 4> &P2,
                                                                             const std::vector<cv::Point2d> &u1,
                                                                             const std::vector<cv::Point2d> &u2);

        [[maybe_unused]] sensor_msgs::PointCloud2 pts_to_cloud(const std::vector<Eigen::Vector3d> &pts);

        [[maybe_unused]] Eigen::Vector3d estimate_point_between_rays(const Eigen::Vector3d &o1,
                                                                     const Eigen::Vector3d &o2,
                                                                     const Eigen::Vector3d &r1,
                                                                     const Eigen::Vector3d &r2);

        [[maybe_unused]] visualization_msgs::Marker create_marker_ray(const Eigen::Vector3d &pt,
                                                                      const Eigen::Vector3d &O,
                                                                      const std::string &cam_name,
                                                                      const int id,
                                                                      const cv::Scalar &color);

        [[maybe_unused]] visualization_msgs::Marker create_marker_pt(const Eigen::Vector3d &pt,
                                                                     const int id,
                                                                     const cv::Scalar &color);

        [[maybe_unused]] void draw_epipolar_line(cv::Mat &img,
                                                 std::vector<cv::Point3f> &line,
                                                 const std::vector<cv::Point2f> &pts);

        [[maybe_unused]] static cv::Scalar generate_random_color();
    };
//}

}  // namespace camera_localization
