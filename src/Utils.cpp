//
// Created by mrs on 22/05/2022.
//

#include "Utils.h"

namespace camera_localization {

    [[maybe_unused]] Eigen::Matrix3d f2K33(const boost::array<double, 12> &P_in) {
        Eigen::Matrix3d K_out = Eigen::Matrix3d::Identity();
        K_out(0, 0) = P_in[0];
        K_out(0, 2) = P_in[2];
        K_out(1, 1) = P_in[5];
        K_out(1, 2) = P_in[6];
        return K_out;
    }

    [[maybe_unused]] Eigen::Vector3d cross(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
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

    [[maybe_unused]] void normalize_point(Eigen::Vector3d &p) {
        p.x() /= p.z();
        p.y() /= p.z();
        p.z() /= p.z();
    }

    [[maybe_unused]] void normalize_line(Eigen::Vector3d &p) {
        auto div = std::sqrt(std::pow(p.x(), 2) + std::pow(p.y(), 2));
        p.x() /= div;
        p.y() /= div;
        p.z() /= div;
    }

    [[maybe_unused]] cv::Point2d PX2u(const Eigen::Matrix<double, 3, 4> &P,
                                      const Eigen::Vector3d &x) {
        Eigen::Matrix<double, 4, 1> homogenous_3d(x.x(), x.y(), x.z(), 1.0);
        Eigen::Vector3d homogenous_2d = P * Eigen::Vector4d{x.x(), x.y(), x.z(), 1};
        normalize_point(homogenous_2d);
        return {homogenous_2d.x(), homogenous_2d.y()};
    }

    [[maybe_unused]] std::vector<Eigen::Vector3d> X2td(const cv::Mat &input) {
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

    [[maybe_unused]] visualization_msgs::Marker create_marker_ray(const Eigen::Vector3d &pt,
                                                                  const Eigen::Vector3d &O,
                                                                  const std::string &cam_name,
                                                                  const int id,
                                                                  const cv::Scalar &color) {
        visualization_msgs::Marker m1;
        m1.header.frame_id = cam_name;
        m1.header.stamp = ros::Time::now();
        m1.points.reserve(2);
        geometry_msgs::Point p1;
        p1.x = pt.x();
        p1.y = pt.y();
        p1.z = pt.z();
        m1.ns = "rays";
        m1.id = id;
        m1.color.a = 1;
        m1.lifetime = ros::Duration(1.0);
        m1.color.r = color[0] / 255.0;
        m1.color.g = color[1] / 255.0;
        m1.color.b = color[2] / 255.0;
        geometry_msgs::Point o;
        o.x = O.x();
        o.y = O.y();
        o.z = O.z();
        m1.points.push_back(o);
        m1.points.push_back(p1);
        m1.type = visualization_msgs::Marker::ARROW;
        m1.scale.x = 0.001;
        m1.scale.y = 0.01;
        m1.scale.z = 0;
        return m1;
    }

    inline auto reprojection_error_half(const Eigen::Matrix<double, 3, 4> &P,
                                        const Eigen::Vector3d &X,
                                        const cv::Point2d &m) {
        Eigen::Matrix<double, 4, 1> x{X.x(), X.y(), X.z(), 1};
        const double a = m.x - P.row(0).dot(x) / P.row(2).dot(x);
        const double b = m.y - P.row(1).dot(x) / P.row(2).dot(x);
        return a * a + b * b;
    }

    [[maybe_unused]] double reprojection_error(const Eigen::Matrix<double, 3, 4> &P1,
                                               const Eigen::Matrix<double, 3, 4> &P2,
                                               const Eigen::Vector3d &X,
                                               const cv::Point2d &m1,
                                               const cv::Point2d &m2) {
        return reprojection_error_half(P1, X, m1) +
               reprojection_error_half(P2, X, m2);
    }
}