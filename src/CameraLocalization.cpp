#include <CameraLocalization.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace camera_localization {

/* onInit() method //{ */
    void CameraLocalization::onInit() {

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

        // initialize cameras roi
        std::vector<int> lroi, rroi;
        pl.loadParam("cam_fleft_roi/x_y_w_h", lroi);
        pl.loadParam("cam_fright_roi/x_y_w_h", rroi);

        // booleans for debug control
        pl.loadParam("debug_sett/debug_images", m_debug_images);
        pl.loadParam("debug_sett/debug_distances", m_debug_distances);
        pl.loadParam("debug_sett/debug_matches", m_debug_matches);
        pl.loadParam("debug_sett/debug_markers", m_debug_markers);

        // image matching and filtering parameters
        int tmp_thr;
        pl.loadParam("corresp/distance_threshold_px", tmp_thr);
        pl.loadParam("corresp/distances_ratio", m_distance_ratio);

        // detector initialization
        int n_features;
        pl.loadParam("corresp/n_features", n_features);

        // load the triangulation method
        pl.loadParam("algorithms/triangulation_method", m_method_triang);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", NODENAME.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", NODENAME.c_str());
        }
        // | ---------------- some data post-processing --------------- |

        detector = cv::ORB::create(n_features);

        // initiate masks for an image matching part
        rect_l = cv::Rect{lroi[0], lroi[1], lroi[2], lroi[3]};
        rect_r = cv::Rect{rroi[0], rroi[1], rroi[2], rroi[3]};
        m_mask_left(rect_l) = cv::Scalar{255};
        m_mask_right(rect_r) = cv::Scalar{255};

        if (tmp_thr < 0) {
            ROS_ERROR("[%s]: wrong distance_threshold_px parameter: should be x > 0", NODENAME.c_str());
        }
        m_distance_threshold = static_cast<size_t>(tmp_thr);
        if ((m_distance_ratio > 1) or (m_distance_ratio < 0)) {
            ROS_ERROR("[%s]: wrong distance_ration parameter: should be 0 < x < 1", NODENAME.c_str());
        }
        if (not(m_method_triang == "svd" or m_method_triang == "primitive")) {
            ROS_ERROR("[%s]: wrong triangulation method", NODENAME.c_str());
        }

        // | ----------------- publishers initialize ------------------ |

        // module results publisher
        m_pub_pcld = nh.advertise<sensor_msgs::PointCloud2>("tdpts", 1, true);

        // images for debug
        if (m_debug_images) {
            m_pub_im_left_debug = nh.advertise<sensor_msgs::Image>("left_debug", 1);
            m_pub_im_right_debug = nh.advertise<sensor_msgs::Image>("right_debug", 1);
        }
        if (m_debug_images) {
            m_pub_markarray = nh.advertise<visualization_msgs::MarkerArray>("markerarray", 1);
        }
        if (m_debug_matches) {
            m_pub_im_corresp = nh.advertise<sensor_msgs::Image>("im_corresp", 1);
        }
        // | ---------------- subscribers initialize ------------------ |

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("CameraLocalization");
        m_transformer.setDefaultPrefix(m_uav_name);

        // | -------------------- initialize timers ------------------- |
//        m_tim_corresp = nh.createTimer(ros::Duration(0.00001),
//                                       &CameraLocalization::m_tim_cbk_corresp,
//                                       this);

        // | -------------------- other static preperation ------------------- |

        mrs_lib::SubscribeHandlerOptions shopt{nh};
        shopt.node_name = NODENAME;
        shopt.threadsafe = true;
        shopt.no_message_timeout = ros::Duration(1.0);

        mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> handler_camleftinfo, handler_camrightinfo;

        mrs_lib::construct_object(m_handler_imleft,
                                  shopt,
                                  "/" + m_uav_name + "/basler_left/image_raw");
        mrs_lib::construct_object(m_handler_imright,
                                  shopt,
                                  "/" + m_uav_name + "/basler_right/image_raw");

        mrs_lib::construct_object(handler_camleftinfo,
                                  shopt,
                                  "/" + m_uav_name + "/basler_left/camera_info");
        mrs_lib::construct_object(handler_camrightinfo,
                                  shopt,
                                  "/" + m_uav_name + "/basler_right/camera_info");
        // initialize cameras with pinhole modeller
        while (not(handler_camleftinfo.newMsg() and handler_camrightinfo.newMsg())) {
            ROS_WARN_THROTTLE(1.0, "[%s]: waiting for camera info messages", NODENAME.c_str());
        }

        m_camera_right.fromCameraInfo(handler_camrightinfo.getMsg());
        m_camera_left.fromCameraInfo(handler_camleftinfo.getMsg());

        m_K_CL_eig = f2K33(handler_camleftinfo.getMsg()->P);
        m_K_CR_eig = f2K33(handler_camrightinfo.getMsg()->P);
        handler_camleftinfo.stop();
        handler_camrightinfo.stop();

        cv::eigen2cv(m_K_CL_eig, m_K_CL_cv);
        cv::eigen2cv(m_K_CR_eig, m_K_CR_cv);

//        find base-to-right camera and base-to-left camera transformations
        ros::Duration(1.0).sleep();
        setUp();
        ROS_INFO_ONCE("[CameraLocalization]: initialized");
        m_is_initialized = true;
    }
//}

    void CameraLocalization::setUp() {

        auto m_fright_pose_opt = m_transformer.getTransform(m_name_CR, m_name_base);
        if (m_fright_pose_opt.has_value()) {
            m_fright_pose = tf2::transformToEigen(m_fright_pose_opt.value());
        } else {
            ROS_ERROR_ONCE("[%s]: No right camera position found.\n", NODENAME.c_str());
            ros::shutdown();
        }

        auto m_fleft_pose_opt = m_transformer.getTransform(m_name_CL, m_name_base);
        if (m_fleft_pose_opt.has_value()) {
            m_fleft_pose = tf2::transformToEigen(m_fleft_pose_opt.value());
        } else {
            ROS_ERROR_ONCE("[%s]: No left camera position found.\n", NODENAME.c_str());
            ros::shutdown();
        }

        auto m_RL_transform_opt = m_transformer.getTransform(m_name_CR, m_name_CL);
        auto m_LR_transform_opt = m_transformer.getTransform(m_name_CL, m_name_CR);

        if (not(m_LR_transform_opt.has_value() and m_RL_transform_opt.has_value())) {
            ROS_ERROR_THROTTLE(2.0, "NO RL OR LR transformation");
            ros::shutdown();
        }
        // initialise transformations
        m_RL_transform = m_RL_transform_opt.value();
        m_LR_transform = m_LR_transform_opt.value();

        m_P_L_eig.topLeftCorner<3, 3>() = m_fleft_pose.inverse().rotation();
        m_P_L_eig.col(3) = m_fleft_pose.inverse().translation();

        m_P_R_eig.topLeftCorner<3, 3>() = m_fright_pose.inverse().rotation();
        m_P_R_eig.col(3) = m_fright_pose.inverse().translation();

        m_P_R_eig = m_K_CR_eig * m_P_R_eig;
        m_P_L_eig = m_K_CL_eig * m_P_L_eig;


        cv::eigen2cv(m_P_L_eig, m_P_L_cv);
        cv::eigen2cv(m_P_R_eig, m_P_R_cv);

        OL_frameR = {m_LR_transform.transform.translation.x,
                     m_LR_transform.transform.translation.y,
                     m_LR_transform.transform.translation.z};

        OR_frameL = {m_RL_transform.transform.translation.x,
                     m_RL_transform.transform.translation.y,
                     m_RL_transform.transform.translation.z};

        m_o1_3d = cv::Point3d{m_fleft_pose.translation().x(),
                              m_fleft_pose.translation().y(),
                              m_fleft_pose.translation().z()};
        m_o2_3d = cv::Point3d{m_fright_pose.translation().x(),
                              m_fright_pose.translation().y(),
                              m_fright_pose.translation().z()};

        m_o1_2d = m_camera_right.project3dToPixel(OL_frameR);
        m_o2_2d = m_camera_left.project3dToPixel(OR_frameL);
    }

// | ---------------------- msg callbacks --------------------- |

// | --------------------- timer callbacks -------------------- |

    void CameraLocalization::m_tim_cbk_corresp([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_is_initialized) return;

        if (m_handler_imleft.newMsg() and m_handler_imright.newMsg()) {

            ROS_INFO_THROTTLE(2.0, "[%s]: looking for correspondences", NODENAME.c_str());

            const auto cv_image_left = cv_bridge::toCvShare(m_handler_imleft.getMsg(), "bgr8").get()->image;
            const auto cv_image_right = cv_bridge::toCvShare(m_handler_imright.getMsg(), "bgr8").get()->image;

            cv::Mat descriptor1, descriptor2;
            std::vector<cv::KeyPoint> keypoints1, keypoints2;

            m_detect_and_compute_kpts(cv_image_left, m_mask_left, keypoints1, descriptor1);
            m_detect_and_compute_kpts(cv_image_right, m_mask_right, keypoints2, descriptor2);
            // detect features and compute correspondances

            if ((keypoints1.size() < 10) or (keypoints2.size() < 10)) {
                ROS_WARN_THROTTLE(1.0, "[%s]: no keypoints visible", NODENAME.c_str());
                return;
            }

            std::vector<cv::DMatch> matches;
            matcher->match(descriptor1,
                           descriptor2,
                           matches,
                           cv::Mat());
            // drop bad matches
            std::sort(matches.begin(), matches.end());
            const int num_good_matches = static_cast<int>(std::round(static_cast<double>(matches.size()) *
                                                                     m_distance_ratio));
            matches.erase(matches.begin() + num_good_matches, matches.end());

            std::vector<cv::DMatch> matches_filtered;
            std::vector<cv::Point2d> kpts_filtered_1, kpts_filtered_2;

            filter_matches(matches,
                           keypoints1, keypoints2,
                           m_o1_2d, m_o2_2d,
                           matches_filtered, kpts_filtered_1, kpts_filtered_2);

            if (m_debug_matches) {
                cv::Mat im_matches;
                cv::drawMatches(cv_image_left, keypoints1,
                                cv_image_right, keypoints2,
                                matches_filtered,
                                im_matches,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                std::vector<char>(),
                                cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                m_pub_im_corresp.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", im_matches).toImageMsg());
                ROS_INFO_THROTTLE(2.0, "[%s & OpenCV]: Correspondences published", NODENAME.c_str());
            }
            auto markerarr = boost::make_shared<visualization_msgs::MarkerArray>();
            Eigen::Vector3d O{0, 0, 0};
            int counter = 0;
            std::vector<Eigen::Vector3d> res_pts_3d;
            if (m_method_triang == "svd") {
                cv::Mat res_4d_homogenous;
                try {
                    cv::triangulatePoints(m_P_L_cv, m_P_R_cv, kpts_filtered_1, kpts_filtered_2, res_4d_homogenous);
                } catch (cv::Exception &e) {
                    std::cout << e.what() << std::endl;
                    return;
                }
                res_pts_3d = X2td(res_4d_homogenous);
            } else if (m_method_triang == "primitive") {
                res_pts_3d = triangulate_primitive(kpts_filtered_1, kpts_filtered_2);
            } else {
                ROS_ERROR("[%s]: unknown triangulation method", NODENAME.c_str());
                ros::shutdown();
            }
            std::vector<cv::Scalar> colors;
            for (size_t i = 0; i < kpts_filtered_1.size(); ++i) {
                const auto color = generate_random_color();
                colors.push_back(color);
            }

            if (m_debug_markers) {
                for (size_t i = 0; i < kpts_filtered_1.size(); ++i) {
                    const auto cv_ray1 = m_camera_left.projectPixelTo3dRay(kpts_filtered_1[i]);
                    const auto cv_ray2 = m_camera_right.projectPixelTo3dRay(kpts_filtered_2[i]);
                    const Eigen::Vector3d eigen_vec1{cv_ray1.x, cv_ray1.y, cv_ray1.z};
                    const Eigen::Vector3d eigen_vec2{cv_ray2.x, cv_ray2.y, cv_ray2.z};

                    markerarr->markers.emplace_back(create_marker_ray(eigen_vec1, O, m_name_CL, counter++, colors[i]));
                    markerarr->markers.emplace_back(create_marker_ray(eigen_vec2, O, m_name_CR, counter++, colors[i]));
                    markerarr->markers.push_back(create_marker_pt(m_name_base, res_pts_3d[i], counter++, colors[i]));
                }
            }
            if (m_debug_distances) {
                cv::Mat imright, imleft;
                cv_image_right.copyTo(imright);
                cv_image_left.copyTo(imleft);
                cv::rectangle(imleft, rect_l, cv::Scalar{0, 100, 0}, 2);
                cv::rectangle(imright, rect_r, cv::Scalar{0, 100, 0}, 2);
                if (m_debug_distances) {
                    for (size_t i = 0; i < res_pts_3d.size(); ++i) {
                        std::ostringstream out;
                        out.precision(2);
                        out << std::fixed << res_pts_3d[i].norm();
                        cv::putText(imleft,
                                    out.str(),
                                    kpts_filtered_1[i],
                                    cv::FONT_HERSHEY_PLAIN,
                                    1,
                                    colors[i],
                                    2);

                        cv::putText(imright,
                                    out.str(),
                                    kpts_filtered_2[i],
                                    cv::FONT_HERSHEY_PLAIN,
                                    1,
                                    colors[i],
                                    2);
                    }
                }
                m_pub_im_left_debug.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", imleft).toImageMsg());
                m_pub_im_right_debug.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", imright).toImageMsg());
            }
            if (m_debug_markers) {
                m_pub_markarray.publish(markerarr);
            }
            auto pc = pts_to_cloud(res_pts_3d, m_name_base);
            m_pub_pcld.publish(pc);
        } else {
            ROS_WARN_THROTTLE(2.0, "[%s]: No new images to search for correspondences", NODENAME.c_str());
        }
    }


// | -------------------- other functions ------------------- |

    // ===================== UTILS =====================
    std::vector<Eigen::Vector3d> CameraLocalization::triangulate_primitive(const std::vector<cv::Point2d> &kpts1,
                                                                           const std::vector<cv::Point2d> &kpts2) {
        std::vector<Eigen::Vector3d> res_pc;
        for (size_t i = 0; i < kpts1.size(); ++i) {
            auto cv_ray1 = m_camera_left.projectPixelTo3dRay(kpts1[i]);
            auto cv_ray2 = m_camera_right.projectPixelTo3dRay(kpts2[i]);

            Eigen::Vector3d eigen_vec1{cv_ray1.x, cv_ray1.y, cv_ray1.z};
            Eigen::Vector3d eigen_vec2{cv_ray2.x, cv_ray2.y, cv_ray2.z};

            auto ray_opt = m_transformer.transformSingle(m_name_CL, eigen_vec1, m_name_base, ros::Time(0));
            auto ray2_opt = m_transformer.transformSingle(m_name_CR, eigen_vec2, m_name_base, ros::Time(0));
            if (ray_opt.has_value() and ray2_opt.has_value()) {
                auto pt = estimate_point_between_rays(NODENAME,
                                                      {m_o1_3d.x, m_o1_3d.y, m_o1_3d.z},
                                                      {m_o2_3d.x, m_o2_3d.y, m_o2_3d.z},
                                                      ray_opt.value(),
                                                      ray2_opt.value());
                res_pc.emplace_back(pt.x(), pt.y(), pt.z());
            }
        }
        return res_pc;
    }

    void CameraLocalization::m_detect_and_compute_kpts(const cv::Mat &img,
                                                       const cv::Mat &mask,
                                                       std::vector<cv::KeyPoint> &res_kpts,
                                                       cv::Mat &res_desk) {
        // Find all kpts on a bw image using the ORB detector
        cv::Mat img_gray;
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

        detector->detectAndCompute(img_gray,
                                   mask,
                                   res_kpts,
                                   res_desk);
    }

    void CameraLocalization::filter_matches(const std::vector<cv::DMatch> &input_matches,
                                            const std::vector<cv::KeyPoint> &kpts1,
                                            const std::vector<cv::KeyPoint> &kpts2,
                                            const cv::Point2d &o1_2d,
                                            const cv::Point2d &o2_2d,
                                            std::vector<cv::DMatch> &res_matches,
                                            std::vector<cv::Point2d> &res_kpts1,
                                            std::vector<cv::Point2d> &res_kpts2) {

        for (const auto &matche: input_matches) {
            const cv::Point2f pt1_2d = kpts1[matche.queryIdx].pt;
            const cv::Point2f pt2_2d = kpts2[matche.trainIdx].pt;
            const cv::Point3d ray1_cv = m_camera_left.projectPixelTo3dRay(pt1_2d);
            const cv::Point3d ray2_cv = m_camera_right.projectPixelTo3dRay(pt2_2d);
            const auto ray1_opt = m_transformer.transformAsVector(Eigen::Vector3d{ray1_cv.x, ray1_cv.y, ray1_cv.z},
                                                                  m_LR_transform);
            const auto ray2_opt = m_transformer.transformAsVector(Eigen::Vector3d{ray2_cv.x, ray2_cv.y, ray2_cv.z},
                                                                  m_RL_transform);

            if (not(ray1_opt.has_value() and ray2_opt.has_value())) {
                ROS_WARN_THROTTLE(2.0, "[%s]: It was not possible to transform a ray", m_uav_name.c_str());
                return;
            }
            Eigen::Vector3d ray1 = ray1_opt.value();
            Eigen::Vector3d ray2 = ray2_opt.value();

            auto p1 = m_camera_right.project3dToPixel({ray1.x(), ray1.y(), ray1.z()});
            auto p2 = m_camera_left.project3dToPixel({ray2.x(), ray2.y(), ray2.z()});

            auto epiline2 = cross({p1.x, p1.y, 1}, {o1_2d.x, o1_2d.y, 1});
            auto epiline1 = cross({p2.x, p2.y, 1}, {o2_2d.x, o2_2d.y, 1});

            normalize_line(epiline1);
            normalize_line(epiline2);

            auto dist1 = std::abs(epiline1.dot(Eigen::Vector3d{pt1_2d.x, pt1_2d.y, 1}));
            auto dist2 = std::abs(epiline2.dot(Eigen::Vector3d{pt2_2d.x, pt2_2d.y, 1}));

            if ((dist1 > m_distance_threshold) or (dist2 > m_distance_threshold)) {
                ROS_WARN_THROTTLE(1.0, "filtered corresp");
                continue;
            }
            res_kpts1.push_back(kpts1[matche.queryIdx].pt);
            res_kpts2.push_back(kpts2[matche.trainIdx].pt);
            res_matches.push_back(matche);
        }
    }
}  // namespace camera_localization

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(camera_localization::CameraLocalization, nodelet::Nodelet)
