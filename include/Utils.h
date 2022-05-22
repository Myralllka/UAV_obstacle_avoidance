//
// Created by mrs on 22/05/2022.
//

#ifndef UAV_CAMERA_LOCALIZATION_UTILS_H
#define UAV_CAMERA_LOCALIZATION_UTILS_H

#include <vector>
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

// Opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace camera_localization {
    template<typename T>
    T deg2rad(const T x) { return x * M_PI / 180; }

    template<typename T>
    T rad2deg(const T x) { return x / M_PI * 180; }

    template<class Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> sqs(const Eigen::MatrixBase<Derived> &vec) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
        return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
                vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
    }

    [[maybe_unused]] double reprojection_error(const Eigen::Matrix<double, 3, 4> &P1,
                                               const Eigen::Matrix<double, 3, 4> &P2,
                                               const Eigen::Vector3d &X,
                                               const cv::Point2d &m1,
                                               const cv::Point2d &m2);


    [[maybe_unused]] Eigen::Matrix3d f2K33(const boost::array<double, 12> &P_in);

    [[maybe_unused]] Eigen::Vector3d cross(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

    [[maybe_unused]] std::pair<cv::Point2d, cv::Point2d> line2image(const cv::Point3d &line, int imwidth);

    [[maybe_unused]] void normalize_point(Eigen::Vector3d &p);

    [[maybe_unused]] void normalize_line(Eigen::Vector3d &p);

    [[maybe_unused]] cv::Point2d PX2u(const Eigen::Matrix<double, 3, 4> &P,
                                      const Eigen::Vector3d &x);

    [[maybe_unused]] std::vector<Eigen::Vector3d> X2td(const cv::Mat &input);

    [[maybe_unused]] visualization_msgs::Marker create_marker_ray(const Eigen::Vector3d &pt,
                                                                  const Eigen::Vector3d &O,
                                                                  const std::string &cam_name,
                                                                  const int id,
                                                                  const cv::Scalar &color);

}


#endif //UAV_CAMERA_LOCALIZATION_UTILS_H
