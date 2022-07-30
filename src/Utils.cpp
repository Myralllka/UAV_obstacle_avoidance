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

    [[maybe_unused]] cv::Point3d cross(const cv::Point2d &a, const cv::Point2d &b) {
        return {a.y - b.y,
                b.x - a.x,
                a.x * b.y - a.y * b.x};
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

    [[maybe_unused]] void normalize_line(cv::Point3d &p) {
        auto div = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));
        p.x /= div;
        p.y /= div;
        p.z /= div;
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

    [[maybe_unused]] std::pair<std::vector<cv::KeyPoint>, cv::Mat>
    det_and_desc_general(const sensor_msgs::Image::ConstPtr &msg,
                         const std::string &im_encoding,
                         const cv::Mat &mask,
                         int n_features) {

        cv::Mat desc_l, img_gray;
        std::vector<cv::KeyPoint> kpts_l;
        const auto cv_image = cv_bridge::toCvShare(msg, im_encoding).get()->image;

        cv::cvtColor(cv_image, img_gray, cv::COLOR_BGR2GRAY);

        auto detector = cv::ORB::create(n_features);
        detector->detectAndCompute(img_gray,
                                   mask,
                                   kpts_l,
                                   desc_l);

        return {kpts_l, desc_l};
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
                p.x = (-d - (i * b) - (c * j)) / a;
                p.y = j;
                p.z = i;
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

    void pts2cloud(const std::vector<Eigen::Vector3d> &pts,
                   boost::shared_ptr<sensor_msgs::PointCloud2> &cloud,
                   const std::string &base) {
        //convert point cloud image to ros message

        //figure out number of points
        size_t numpoints = pts.size();

        //declare message and sizes
        cloud->header.frame_id = base;
        cloud->header.stamp = ros::Time::now();
        cloud->width = numpoints;
        cloud->height = 1;
        cloud->is_bigendian = false;
        cloud->is_dense = false; // there may be invalid points

        //for fields setup
        sensor_msgs::PointCloud2Modifier modifier(*cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(numpoints);

        //iterators
        sensor_msgs::PointCloud2Iterator<float> out_x(*cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*cloud, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*cloud, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*cloud, "b");

        for (size_t i = 0; i < pts.size(); ++i, ++out_x, ++out_y, ++out_z, ++out_r, ++out_g, ++out_b) {
            //get the image coordinate for this point and convert to mm
            auto pointcoord = pts[i];
            const float X_World = pointcoord.x();
            const float Y_World = pointcoord.y();
            const float Z_World = pointcoord.z();
            //store xyz in point cloud, transforming from image coordinates, (Z Forward to X Forward)
            *out_x = X_World;
            *out_y = Y_World;
            *out_z = Z_World;
            *out_r = 66;
            *out_g = 66;
            *out_b = 66;
        }
    }

    visualization_msgs::Marker
    create_marker_pt(const std::string &base,
                     const Eigen::Vector3d &pt,
                     const int id,
                     const cv::Scalar &color) {
        visualization_msgs::Marker m1;
        m1.header.frame_id = base;
        m1.header.stamp = ros::Time::now();
        m1.ns = "points";
        m1.id = id;
        m1.lifetime = ros::Duration(1.0);
        m1.color.a = 1;
        m1.color.r = color[0] / 255.0;
        m1.color.g = color[1] / 255.0;
        m1.color.b = color[2] / 255.0;
        m1.pose.position.x = pt.x();
        m1.pose.position.y = pt.y();
        m1.pose.position.z = pt.z();
        m1.type = visualization_msgs::Marker::SPHERE;
        m1.scale.x = 0.1;
        m1.scale.y = 0.1;
        m1.scale.z = 0.1;
        return m1;
    }

    std::vector<Eigen::Vector3d> triangulate_tdv(const Eigen::Matrix<double, 3, 4> &P1,
                                                 const Eigen::Matrix<double, 3, 4> &P2,
                                                 const std::vector<cv::Point2d> &u1,
                                                 const std::vector<cv::Point2d> &u2) {
        std::vector<Eigen::Vector3d> res;
        Eigen::Matrix<double, 4, 4> D;
        for (size_t i = 0; i < u1.size(); ++i) {
            D.row(0) = P1.row(2) * u1[i].x - P1.row(0);
            D.row(1) = P1.row(2) * u1[i].y - P1.row(1);
            D.row(2) = P2.row(2) * u2[i].x - P2.row(0);
            D.row(3) = P2.row(2) * u2[i].y - P2.row(1);
            Eigen::JacobiSVD<Eigen::Matrix<double, 4, 4>, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(D);
            auto X = svd.matrixU().transpose().bottomRows<1>();
            res.emplace_back(X.x() / X.w(), X.y() / X.w(), X.z() / X.w());
        }
        return res;
    }

    [[maybe_unused]] cv::Scalar generate_random_color() {
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_int_distribution<uint8_t> distribution{0, 255};

        uint8_t r = distribution(generator);
        uint8_t g = distribution(generator);
        uint8_t b = distribution(generator);
        return cv::Scalar(b, g, r);
    }

    [[maybe_unused]] void draw_epipolar_line(cv::Mat &img,
                                             std::vector<cv::Point3f> &line,
                                             const std::vector<cv::Point2f> &pts) {
        // source https://docs.opencv.org/3.4/da/de9/tutorial_py_epipolar_geometry.html
        for (size_t i = 0; i < line.size(); ++i) {
            auto w = img.size[1]; // c
            // randomly generate line color
            auto color = generate_random_color();
            auto x0 = .0f;
            auto x1 = static_cast<double>(w);

            double l0 = line[i].x;
            double l1 = line[i].y;
            double l2 = line[i].z;

            double y0 = -l2 / l1;
            double y1 = -(l2 + l0 * w) / l1;

            auto p1 = cv::Point{static_cast<int>(std::round(x0)), static_cast<int>(std::ceil(y0))};
            auto p2 = cv::Point{static_cast<int>(std::round(x1)), static_cast<int>(std::ceil(y1))};

            cv::line(img, p1, p2, color, 3);
            cv::circle(img, pts[i], 2, color, 5);
        }
    }

    Eigen::Vector3d estimate_point_between_rays(const std::string &nodename,
                                                const Eigen::Vector3d &o1,
                                                const Eigen::Vector3d &o2,
                                                const Eigen::Vector3d &r1,
                                                const Eigen::Vector3d &r2) {
        const Eigen::Vector3d m = o2 - o1;
        const Eigen::Vector3d d1 = r1.normalized();
        const Eigen::Vector3d d2 = r2.normalized();

        const double s = std::abs((d2.dot(m) - d1.dot(m) * d1.dot(d2)) / (1.0 - d1.dot(d2) * d1.dot(d2)));
        const double t = std::abs((d1.dot(m) + s * d1.dot(d2)));

        const Eigen::Vector3d p1 = o1 + t * d1;
        const Eigen::Vector3d p2 = o2 + s * d2;

        Eigen::Vector3d res = (p1 + p2) / 2;
        res = p2;

        ROS_INFO_THROTTLE(2.0, "[%s]: closest distance of rays: %.2fm",
                          nodename.c_str(),
                          (p2 - p1).norm());
        ROS_INFO_THROTTLE(2.0, "[%s]: distance to the object: %.2fm",
                          nodename.c_str(),
                          res.norm());
        return res;
    }
}