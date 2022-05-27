//
// Created by mrs on 22/05/2022.
//

#include "Utils.h"

namespace camera_localization {

    [[maybe_unused]] double dist_plane2pt(const Eigen::Vector4d &plane,
                                          const Eigen::Vector3d &pt) {
        return std::abs(plane.x() * pt.x() + plane.y() * pt.y() + plane.z() * pt.z() + plane.w()) /
               std::sqrt(std::pow(plane.x(), 2) + std::pow(plane.y(), 2) + std::pow(plane.z(), 2));
    }

    [[maybe_unused]] double dist_plane2pt(const Eigen::Vector4d &plane,
                                          const pcl::PointXYZ &pt) {
        return std::abs(plane.x() * pt.x + plane.y() * pt.y + plane.z() * pt.z + plane.w()) /
               std::sqrt(std::pow(plane.x(), 2) + std::pow(plane.y(), 2) + std::pow(plane.z(), 2));
    }

    [[maybe_unused]] Eigen::Vector3d closest_pt(const Eigen::Vector4d &plane,
                                                const Eigen::Vector3d &pt) {
        const auto d = dist_plane2pt(plane, pt);
        const Eigen::Vector3d n{plane.x(), plane.y(), plane.z()};
        return pt + (d / n.norm()) * n;
    }

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

    [[maybe_unused]] visualization_msgs::MarkerArray create_marker_plane(const Eigen::Vector4d &plane_eq,
                                                                         const std::string &base_name,
                                                                         const cv::Scalar &color) {
        const double a = plane_eq.x();
        const double b = plane_eq.y();
        const double c = plane_eq.z();
        const double d = plane_eq.w();
        visualization_msgs::MarkerArray res;
        visualization_msgs::Marker m1;
        m1.header.frame_id = base_name;
        m1.header.stamp = ros::Time::now();
        m1.ns = "plane";
        m1.color.a = 1;
        m1.lifetime = ros::Duration(1.0);
        m1.pose.orientation.x = 0.0;
        m1.pose.orientation.y = 0.0;
        m1.pose.orientation.z = 0.0;
        m1.pose.orientation.w = 1.0;
        m1.color.r = color[0] / 255.0;
        m1.color.g = color[1] / 255.0;
        m1.color.b = color[2] / 255.0;
        m1.type = visualization_msgs::Marker::POINTS;
        m1.scale.x = 0.01;
        m1.scale.y = 0.01;
        m1.scale.z = 0;
        const float sss = 1;
        float i = -sss;
        float j;
        while (i <= sss) {
            j = -sss;
            while (j <= sss) {
                geometry_msgs::Point p;
                p.x = i;
                p.y = j;
                p.z = (-d - (i * a) - (b * j)) / c;
                j += 0.05;
                m1.points.push_back(p);
            }
            i += 0.005;
        }
        res.markers.push_back(m1);
        return res;
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
        return std::pow(reprojection_error_half(P1, X, m1), 2) +
               std::pow(reprojection_error_half(P2, X, m2), 2);
    }

    [[maybe_unused]] std::pair<Eigen::Vector3d, Eigen::Vector3d>
    best_plane_from_points_SVD(const std::vector<Eigen::Vector3d> &c) {
        // copy coordinates to  matrix in Eigen format
        size_t num_atoms = c.size();
        Eigen::Matrix<Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_atoms);
        for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

        // calculate centroid
        Eigen::Vector3d centroid(coord.row(0).mean(),
                                 coord.row(1).mean(),
                                 coord.row(2).mean());
        // subtract centroid
        coord.row(0).array() -= centroid(0);
        coord.row(1).array() -= centroid(1);
        coord.row(2).array() -= centroid(2);

        // we only need the left-singular matrix here
        //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
        auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Vector3d plane_normal = svd.matrixU().rightCols<1>();
        return std::make_pair(centroid, plane_normal);
    }

    [[maybe_unused]] std::optional<std::pair<Eigen::Vector4d, std::vector<int>>>
    best_plane_from_points_RANSAC(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_MLESAC);
        seg.setDistanceThreshold(0.02);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
            return {};
        }

        return std::pair{Eigen::Vector4d{coefficients->values[0],
                                         coefficients->values[1],
                                         coefficients->values[2],
                                         coefficients->values[3]},
                         inliers->indices};
    }
}