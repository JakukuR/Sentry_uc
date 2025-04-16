// Copyright 2025 zjl

#ifndef ARMOR_DETECTOR__ANGLE_OPTIMIZER_HPP_
#define ARMOR_DETECTOR__ANGLE_OPTIMIZER_HPP_


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core.hpp>
#include <Eigen/Dense>
// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/pnp_solver.hpp"
namespace rm_auto_aim
{


cv::Vec2d rotate(const cv::Vec2d& vector, double angle);
cv::Mat reduced_angle(cv::Mat angle);

// class SingleCost {
// public:
//     explicit SingleCost(const PnPSolver& solver, const Armor& armor, const double& z_to_v_exp);
//     double operator()(const double& x);

// private:
//     const PnPSolver& solver_;
//     const Armor& armor_;
//     const double z_to_v_exp_;
// };


class AngleOptimizer {
public:
    // 定义装甲板尺寸
    static constexpr float SMALL_ARMOR_WIDTH = 0.135;
    static constexpr float SMALL_ARMOR_HEIGHT = 0.055;
    static constexpr float LARGE_ARMOR_WIDTH = 0.225;
    static constexpr float LARGE_ARMOR_HEIGHT = 0.055;

    // AngleOptimizer();
    AngleOptimizer(
        const std::array<double, 9> & camera_matrix,
        const std::vector<double> & distortion_coefficients);
        std::vector<cv::Point2f> image_pts;

    std::vector<cv::Point2f> image_armor_points = {};
    
    std::vector<cv::Point3f> radial_armor_corners(
        const Armor armor
    ); 
    std::vector<cv::Point2f> radial_armor_pts(
        std::vector<cv::Point3f> world_points,
        const cv::Mat &tvec,
        const cv::Mat &rvec
    ); 
    geometry_msgs::msg::PoseStamped odem_to_cameraopf(
        const std::shared_ptr<tf2_ros::Buffer> &tf2_buffer,
        geometry_msgs::msg::PoseStamped &transformed_pose,
        double &yaw,
        double &pitch,
        cv::Mat &rvec
    );
    double ternary_search_yaw(
        const Armor armor,
        cv::Mat tvec,
        cv::Mat rvec,
        const std::vector<cv::Point3f> &world_points,
        const std::shared_ptr<tf2_ros::Buffer> &tf2_buffer_,
        geometry_msgs::msg::PoseStamped &transformed_pose
    );
    double cal_cost(
        const Armor armor,
        cv::Mat tvec,
        cv::Mat rvec,
        const std::vector<cv::Point3f> &world_points,
        const std::shared_ptr<tf2_ros::Buffer> &tf2_buffer_,
        geometry_msgs::msg::PoseStamped &transformed_pose,
        double& x
    );
    
    double get_pts_cost(
        const std::vector<cv::Point2f>& cv_refs,
        const std::vector<cv::Point2f>& cv_pts,
        const double& inclined
    );

    double sq(double value);
    double get_abs_angle(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);
private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    // const PnPSolver& solver_;
    // const Armor& armor_;
    // const double z_to_v_exp_;

    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> large_armor_points_;
};
}
#endif