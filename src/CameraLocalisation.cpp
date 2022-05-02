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
        mrs_lib::ParamLoader pl(nh, NODENAME);

        pl.loadParam("UAV_NAME", m_uav_name);

        pl.loadParam("base_frame_pose", m_name_base);
        pl.loadParam("m_name_CL", m_name_CL);
        pl.loadParam("m_name_CR", m_name_CR);
        pl.loadParam("cam_fleft_roi/start_x", m_fleft_roi.x);
        pl.loadParam("cam_fleft_roi/start_y", m_fleft_roi.y);
        pl.loadParam("cam_fleft_roi/h", m_fleft_roi.height);
        pl.loadParam("cam_fleft_roi/w", m_fleft_roi.width);
        pl.loadParam("cam_fright_roi/start_x", m_fright_roi.x);
        pl.loadParam("cam_fright_roi/start_y", m_fright_roi.y);
        pl.loadParam("cam_fright_roi/h", m_fright_roi.height);
        pl.loadParam("cam_fright_roi/w", m_fright_roi.width);
        pl.loadParam("corresp/debug_epipolar", m_debug_epipolar);
        pl.loadParam("corresp/debug_matches", m_debug_matches);

        int tmp_thr;
        pl.loadParam("corresp/distance_threshold_px", tmp_thr);
        if (tmp_thr < 0) {
            ROS_INFO_ONCE("[%s]: wrong distance_threshold_px parameter: should be x > 0", NODENAME.c_str());
        }
        m_distance_threshold = static_cast<size_t>(tmp_thr);
//        5375414116326520
        pl.loadParam("corresp/distances_ratio", m_distance_ratio);
        if ((m_distance_ratio > 1) or (m_distance_ratio < 0)) {
            ROS_INFO_ONCE("[%s]: wrong distance_ration parameter: should be 0 < x < 1", NODENAME.c_str());
        }
        int n_features;
        pl.loadParam("corresp/n_features", n_features);
        detector = cv::ORB::create(n_features);
        // intrinsic camera parameters (calibration matrices)
        Eigen::Matrix<double, 3, 3> K_L, K_R;
//        K_L = pl.loadMatrixStatic2<3, 3>("basler_left/camera_matrix/data");
//        K_R = pl.loadMatrixStatic2<3, 3>("basler_right/camera_matrix/data");
        cv::eigen2cv(K_L, m_K_CL);
        cv::eigen2cv(K_R, m_K_CR);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[CameraLocalisation]: failed to load non-optional parameters!");
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[CameraLocalisation]: loaded parameters");
        }
        // | ---------------- some data post-processing --------------- |

        // | ----------------- publishers initialize ------------------ |

        m_pub_im_left_epipolar = nh.advertise<sensor_msgs::Image>("epimleft", 1);
        m_pub_im_right_epipolar = nh.advertise<sensor_msgs::Image>("epimright", 1);
        m_pub_pcld = nh.advertise<sensor_msgs::PointCloud2>("tdpts", 1, true);
        m_pub_markarray = nh.advertise<visualization_msgs::MarkerArray>("markerarray", 1);
        m_pub_im_corresp = nh.advertise<sensor_msgs::Image>("im_corresp", 1);
        // | ---------------- subscribers initialize ------------------ |

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("CameraLocalisation");
        m_transformer.setDefaultPrefix(m_uav_name);

        // | -------------------- initialize timers ------------------- |
        m_tim_corresp = nh.createTimer(ros::Duration(0.0001),
                                       &CameraLocalisation::m_tim_cbk_corresp,
                                       this);

        // | -------------------- other static preperation ------------------- |

        mrs_lib::SubscribeHandlerOptions shopt{nh};
        shopt.node_name = NODENAME;
        shopt.threadsafe = true;
        shopt.no_message_timeout = ros::Duration(1.0);
        mrs_lib::construct_object(m_handler_imleft,
                                  shopt,
//                                      "/" + m_uav_name + "/fleft/tag_detections_image");
                                  "/" + m_uav_name + "/basler_left/image_rect");
        mrs_lib::construct_object(m_handler_imright,
                                  shopt,
//                                      "/" + m_uav_name + "/fright/tag_detections_image");
                                  "/" + m_uav_name + "/basler_right/image_rect");
        mrs_lib::construct_object(m_handler_camleftinfo,
                                  shopt,
                                  "/" + m_uav_name + "/basler_left/camera_info");
        mrs_lib::construct_object(m_handler_camrightinfo,
                                  shopt,
                                  "/" + m_uav_name + "/basler_right/camera_info");
        // initialize cameras with pinhole modeller
        while (not(m_handler_camleftinfo.newMsg() and m_handler_camrightinfo.newMsg())) {
            ROS_WARN_THROTTLE(1.0, "[%s]: waiting for camera info messages", NODENAME.c_str());
        }

        m_camera_right.fromCameraInfo(m_handler_camrightinfo.getMsg());
        m_camera_left.fromCameraInfo(m_handler_camleftinfo.getMsg());

        // initiate masks for an image matching part
        mask_left(cv::Rect{m_fleft_roi.x,
                           m_fleft_roi.y,
                           m_fleft_roi.width,
                           m_fleft_roi.height}) = cv::Scalar{255};
        mask_right(cv::Rect{m_fright_roi.x,
                            m_fright_roi.y,
                            m_fright_roi.width,
                            m_fright_roi.height}) = cv::Scalar{255};
//        find base-to-right transformation
        ros::Duration(1.0).sleep();
        auto m_fright_pose_opt = m_transformer.getTransform(m_name_CR, m_name_base);
        if (m_fright_pose_opt.has_value()) {
            m_fright_pose = tf2::transformToEigen(m_fright_pose_opt.value());
        } else {
            ROS_ERROR_ONCE("[%s]: fuck. No right camera position found.\n", NODENAME.c_str());
            ros::shutdown();
        }

        // initialise transformations
        m_eig_P_L.topLeftCorner<3, 3>() = m_fleft_pose.rotation();
        m_eig_P_L.col(3) = m_fleft_pose.translation();
        m_eig_P_R.topLeftCorner<3, 3>() = m_fright_pose.rotation();
        m_eig_P_R.col(3) = m_fright_pose.translation();

        m_eig_P_R = K_R * m_eig_P_R;
        m_eig_P_L = K_L * m_eig_P_L;

        cv::eigen2cv(m_eig_P_L, m_P_L);
        cv::eigen2cv(m_eig_P_R, m_P_R);

        ROS_INFO_ONCE("[CameraLocalisation]: initialized");

        m_is_initialized = true;

    }
//}


// | ---------------------- msg callbacks --------------------- |

// | --------------------- timer callbacks -------------------- |

    void CameraLocalisation::m_tim_cbk_corresp([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_is_initialized) return;

        if (m_handler_imleft.newMsg() and m_handler_imright.newMsg()) {
            auto RL = m_transformer.getTransform(m_name_CR,
                                                 m_name_CL);

            auto LR = m_transformer.getTransform(m_name_CL,
                                                 m_name_CR);

            if (not(LR.has_value() and RL.has_value())) {
                ROS_WARN_THROTTLE(2.0, "NO RL OR LR transformation");
                return;
            } else {
                ROS_INFO_THROTTLE(2.0, "[%s]: looking for correspondences", NODENAME.c_str());
            }

            cv::Point3d origin1{LR.value().transform.translation.x,
                                LR.value().transform.translation.y,
                                LR.value().transform.translation.z};

            cv::Point3d origin2{RL.value().transform.translation.x,
                                RL.value().transform.translation.y,
                                RL.value().transform.translation.z};

            cv::Point2d o1 = m_camera_right.project3dToPixel(origin1);
            cv::Point2d o2 = m_camera_left.project3dToPixel(origin2);
            // use const + toCvShared
            auto cv_image_left = cv_bridge::toCvCopy(m_handler_imleft.getMsg(),
                                                     "bgr8").get()->image;
            auto cv_image_right = cv_bridge::toCvCopy(m_handler_imright.getMsg(),
                                                      "bgr8").get()->image;
            cv::Mat im_gray_left, im_gray_right;
            cv::cvtColor(cv_image_left, im_gray_left, cv::COLOR_BGR2GRAY);
            cv::cvtColor(cv_image_right, im_gray_right, cv::COLOR_BGR2GRAY);

            cv::Mat descriptor1, descriptor2;
            std::vector<cv::KeyPoint> keypoints1, keypoints2;
            // detect features and compute correspondances
            detector->detectAndCompute(im_gray_left,
                                       mask_left,
                                       keypoints1,
                                       descriptor1);
            detector->detectAndCompute(im_gray_right,
                                       mask_right,
                                       keypoints2,
                                       descriptor2);
            [[maybe_unused]] auto w = im_gray_left.size[1];
            if ((keypoints1.size() < 10) or (keypoints2.size() < 10)) {
                ROS_WARN_THROTTLE(1.0, "[%s]: no keypoints visible", NODENAME.c_str());
                return;
            }

            std::vector<cv::DMatch> matches;
            matcher->match(descriptor1,
                           descriptor2,
                           matches,
                           cv::Mat());

            std::sort(matches.begin(), matches.end());
            const int num_good_matches = static_cast<int>(std::round(
                    static_cast<double>(matches.size()) * m_distance_ratio));
            matches.erase(matches.begin() + num_good_matches, matches.end());

            std::vector<cv::DMatch> matches_filtered;

            std::vector<cv::Point2d> kpts_filtered_1, kpts_filtered_2;
            for (auto &matche: matches) {
                cv::Point2f pt1_cv = keypoints1[matche.queryIdx].pt;
                cv::Point2f pt2_cv = keypoints2[matche.trainIdx].pt;
                cv::Point3d ray1_cv = m_camera_left.projectPixelTo3dRay(pt1_cv);
                cv::Point3d ray2_cv = m_camera_right.projectPixelTo3dRay(pt2_cv);
                const auto ray1_opt = m_transformer.transform(ray1_cv, LR.value());
                const auto ray2_opt = m_transformer.transform(ray2_cv, RL.value());

                if (not(ray1_opt.has_value() and ray2_opt.has_value())) {
                    ROS_WARN_THROTTLE(2.0, "[%s]: It was not possible to transform a ray", m_uav_name.c_str());
                    return;
                }
                ray1_cv = ray1_opt.value();
                ray2_cv = ray2_opt.value();

                auto epiline2 = cross(m_camera_right.project3dToPixel(ray1_cv), o1);
                auto epiline1 = cross(m_camera_left.project3dToPixel(ray2_cv), o2);

                normalize_line(epiline1);
                normalize_line(epiline2);

                auto dist1 = std::abs(epiline1.dot(cv::Point3d{pt1_cv.x, pt1_cv.y, 1}));
                auto dist2 = std::abs(epiline2.dot(cv::Point3d{pt2_cv.x, pt2_cv.y, 1}));

                if ((dist1 > m_distance_threshold) or (dist2 > m_distance_threshold)) {
                    ROS_WARN_THROTTLE(1.0, "filtered corresp");
                    continue;
                }
                kpts_filtered_1.push_back(keypoints1[matche.queryIdx].pt);
                kpts_filtered_2.push_back(keypoints2[matche.trainIdx].pt);
                matches_filtered.push_back(matche);
//                if (m_debug_epipolar) {
//                    // Draw epipolar lines
//                    auto color = generate_random_color();
//                    cv::circle(cv_image_left, pt1_cv, 3, color, 3);
//                    cv::circle(cv_image_right, pt2_cv, 3, color, 3);
//                    auto image_pts = line2image(epiline2, w);
//                    auto p1 = image_pts.first;
//                    auto p2 = image_pts.second;
//                    cv::line(cv_image_right, p1, p2, color, 2);
//
//                    image_pts = line2image(epiline1, w);
//                    p1 = image_pts.first;
//                    p2 = image_pts.second;
//                    cv::line(cv_image_left, p1, p2, color, 2);
//
//                    m_pub_im_left_epipolar.publish(
//                            cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_left).toImageMsg());
//                    m_pub_im_right_epipolar.publish(
//                            cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_right).toImageMsg());
//                    ROS_INFO_THROTTLE(2.0, "published epipolars");
//                }
            }
            if (m_debug_matches) {
                cv::Mat im_matches;
                cv::drawMatches(cv_image_left,
                                keypoints1,
                                cv_image_right,
                                keypoints2,
                                matches_filtered,
                                im_matches,
                                cv::Scalar::all(-1),
                                cv::Scalar::all(-1),
                                std::vector<char>(),
                                cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                m_pub_im_corresp.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", im_matches).toImageMsg());
                ROS_INFO_THROTTLE(2.0, "[%s & OpenCV]: Correspondences published", NODENAME.c_str());
            }
            auto markerarr = boost::make_shared<visualization_msgs::MarkerArray>();
            geometry_msgs::Point O1, O2;
            O1.x = 0;
            O1.y = 0;
            O1.z = 0;
            O2.x = 0;
            O2.y = 0;
            O2.z = 0;
            int counter = 0;
            // TODO: fixit
            // triangulate points
            // vector of cv::Point_<double, 3>
//            cv::Mat res_pts_4d;
//            std::vector<cv::Point3d> res_pts_3d;
//            try {
//                res_pts_3d = triangulate_points(m_eig_P_L, m_eig_P_R, kpts_filtered_1, kpts_filtered_2);
//                cv::triangulatePoints(m_P_L, m_P_R, kpts_filtered_1, kpts_filtered_2, res_pts_4d);
//                std::cout << res_pts_4d << std::endl;
//                for (int i = 0; i < res_pts_4d.cols; ++i) {
//                    res_pts_3d[i].x = res_pts_4d.at<double>(i, 0) / res_pts_4d.at<double>(i, 3);
//                    res_pts_3d[i].y = res_pts_4d.at<double>(i, 1) / res_pts_4d.at<double>(i, 3);
//                    res_pts_3d[i].z = res_pts_4d.at<double>(i, 2) / res_pts_4d.at<double>(i, 3);
//                }
//                std::cout << res_pts_3d << std::endl;;
//                auto pc = pts_to_cloud(res_pts_3d);
//                m_pub_pcld.publish(pc);
//            } catch (const cv::Exception &e) {
//                ROS_ERROR("[%s]: %s", NODENAME.c_str(), e.what());
//            }

//        Primitive triangulation

            auto or1 = cv::Point3d{m_fleft_pose.translation().x(),
                                   m_fleft_pose.translation().y(),
                                   m_fleft_pose.translation().z()};
            auto or2 = cv::Point3d{m_fright_pose.translation().x(),
                                   m_fright_pose.translation().y(),
                                   m_fright_pose.translation().z()};
            std::vector<cv::Point3d> res_pts_3d;
            for (size_t i = 0; i < kpts_filtered_1.size(); ++i) {
                auto color = generate_random_color();
                auto cv_ray1 = m_camera_left.projectPixelTo3dRay(kpts_filtered_1[i]);
                auto cv_ray2 = m_camera_right.projectPixelTo3dRay(kpts_filtered_2[i]);

                Eigen::Vector3d eigen_vec1{cv_ray1.x, cv_ray1.y, cv_ray1.z};
                Eigen::Vector3d eigen_vec2{cv_ray2.x, cv_ray2.y, cv_ray2.z};

                auto ray_opt = m_transformer.transformSingle(m_name_CL,
                                                             eigen_vec1,
                                                             m_name_base,
                                                             ros::Time(0));
                auto ray2_opt = m_transformer.transformSingle(m_name_CR,
                                                              eigen_vec2,
                                                              m_name_base,
                                                              ros::Time(0));
                if (ray_opt.has_value() and ray2_opt.has_value()) {
                    auto pt = estimate_point_between_rays({or1.x, or1.y, or1.z},
                                                          {or2.x, or2.y, or2.z},
                                                          ray_opt.value(),
                                                          ray2_opt.value());
                    res_pts_3d.emplace_back(pt.x(), pt.y(), pt.z());
                    res_pts_3d.emplace_back(ray_opt->x(), ray_opt->y(), ray_opt->z());
                    res_pts_3d.emplace_back(ray2_opt->x(), ray2_opt->y(), ray2_opt->z());
                    markerarr->markers.emplace_back(create_marker_ray(ray_opt.value(),
                                                                      O1,
                                                                      m_name_base,
                                                                      counter++,
                                                                      color));
                    markerarr->markers.emplace_back(create_marker_ray(ray2_opt.value(),
                                                                      O2,
                                                                      m_name_base,
                                                                      counter++,
                                                                      color));
//                    markerarr->markers.push_back(create_marker_pt(pt,
//                                                                  counter++,
//                                                                  color));
                }
            }
            m_pub_markarray.publish(markerarr);
            res_pts_3d.push_back(or1);
            res_pts_3d.push_back(or2);
            auto pc = pts_to_cloud(res_pts_3d);
            m_pub_pcld.publish(pc);
        } else {
            ROS_WARN_THROTTLE(2.0, "[%s]: No new images to search for correspondences", NODENAME.c_str());
        }
    }


// | -------------------- other functions ------------------- |

    // ===================== UTILS =====================

    Eigen::Vector3d CameraLocalisation::estimate_point_between_rays(const Eigen::Vector3d &o1,
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
                          NODENAME.c_str(),
                          (p2 - p1).norm());
        ROS_INFO_THROTTLE(2.0, "[%s]: distance to the object: %.2fm",
                          NODENAME.c_str(),
                          res.norm());
        return res;
    }

    visualization_msgs::Marker
    CameraLocalisation::create_marker_ray(const Eigen::Vector3d &pt,
                                          const geometry_msgs::Point O,
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
//        m1.action = visualization_msgs::Marker::ADD;
        m1.color.a = 1;
        m1.lifetime = ros::Duration(1.0);
        m1.color.r = color[0] / 255.0;
        m1.color.g = color[1] / 255.0;
        m1.color.b = color[2] / 255.0;
        m1.points.push_back(O);
        m1.points.push_back(p1);
        m1.type = visualization_msgs::Marker::ARROW;
        m1.scale.x = 0.001;
        m1.scale.y = 0.01;
        m1.scale.z = 0;
        return m1;
    }

    visualization_msgs::Marker
    CameraLocalisation::create_marker_pt(const cv::Point3d &pt,
                                         const int id,
                                         const cv::Scalar &color) {
        visualization_msgs::Marker m1;
        m1.header.frame_id = m_name_base;
        m1.header.stamp = ros::Time::now();
        m1.ns = "points";
        m1.id = id;
        m1.color.a = 1;
        m1.lifetime = ros::Duration(1.0);
        m1.color.r = color[0] / 255.0;
        m1.color.g = color[1] / 255.0;
        m1.color.b = color[2] / 255.0;
        m1.pose.position.x = pt.x;
        m1.pose.position.y = pt.y;
        m1.pose.position.z = pt.z;
        std::cout << m1.points[0] << std::endl;
        m1.type = visualization_msgs::Marker::SPHERE;
        m1.scale.x = 0.1;
        m1.scale.y = 0.1;
        m1.scale.z = 0.1;
        return m1;
    }

    std::vector<cv::Point3d> CameraLocalisation::triangulate_points(const Eigen::Matrix<double, 3, 4> &P1,
                                                                    const Eigen::Matrix<double, 3, 4> &P2,
                                                                    const std::vector<cv::Point2d> &u1,
                                                                    const std::vector<cv::Point2d> &u2) {
        std::vector<cv::Point3d> res;
        Eigen::Matrix<double, 4, 4> D;
        for (size_t i = 0; i < u1.size(); ++i) {
            D.row(0) = P1.row(2) * u1[i].x - P1.row(0);
            D.row(1) = P1.row(2) * u1[i].y - P1.row(1);
            D.row(2) = P2.row(2) * u2[i].x - P2.row(0);
            D.row(3) = P2.row(2) * u2[i].y - P2.row(1);
            Eigen::JacobiSVD<Eigen::Matrix<double, 4, 4>, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(D);
            auto X = svd.matrixV().bottomRows<1>();
            std::cout << X << std::endl;
            res.emplace_back(X.x() / X.w(), X.y() / X.w(), X.z() / X.w());
        }
        return res;
    }

    //convert point cloud image to ros message
    sensor_msgs::PointCloud2 CameraLocalisation::pts_to_cloud(const std::vector<cv::Point3d> &pts) {
        //rgb is a cv::Mat with 3 color channels of size 640x480
        //coords is a cv::Mat with xyz channels of size 640x480, units in mm from calibration

        //figure out number of points
        size_t numpoints = pts.size();

        //declare message and sizes
        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = m_name_base;
        cloud.header.stamp = ros::Time::now();
        cloud.width = numpoints;
        cloud.height = 1;
        cloud.is_bigendian = false;
        cloud.is_dense = false; // there may be invalid points

        //for fields setup
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(numpoints);

        //iterators
        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud, "b");

        for (size_t i = 0; i < pts.size(); ++i, ++out_x, ++out_y, ++out_z, ++out_r, ++out_g, ++out_b) {
            //get the image coordinate for this point and convert to mm
            cv::Point3d pointcoord = pts[i];
            float X_World = pointcoord.x;
            float Y_World = pointcoord.y;
            float Z_World = pointcoord.z;
            //store xyz in point cloud, transforming from image coordinates, (Z Forward to X Forward)
            *out_x = X_World;
            *out_y = Y_World;
            *out_z = Z_World;
            *out_r = 66;
            *out_g = 66;
            *out_b = 66;
        }
        return cloud;
    }

    [[maybe_unused]] cv::Scalar CameraLocalisation::generate_random_color() {
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_int_distribution<uint8_t> distribution{0, 255};

        uint8_t r = distribution(generator);
        uint8_t g = distribution(generator);
        uint8_t b = distribution(generator);
        return cv::Scalar(b, g, r);
    }

    [[maybe_unused]] void CameraLocalisation::draw_epipolar_line(cv::Mat &img,
                                                                 std::vector<cv::Point3f> &line,
                                                                 const std::vector<cv::Point2f> &pts) {
        // source https://docs.opencv.org/3.4/da/de9/tutorial_py_epipolar_geometry.html
        auto w = img.size[1]; // c
        for (size_t i = 0; i < line.size(); ++i) {
//        for (size_t i = 0; i < 100; i += 20) {
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

}  // namespace camera_localisation

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(camera_localisation::CameraLocalisation, nodelet::Nodelet)
