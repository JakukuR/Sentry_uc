// Copyright (c) 2025 zjl
#include <opencv2/calib3d.hpp>
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <cstdio>

#include "armor_detector/angle_optimizer.hpp"

const int FIND_ANGLE_ITERATIONS = 12; // 三分法迭代次数 理想精度 < 1.
const double SIMPLE_TOP_TRACK_AREA_RATIO = 2.;
const double DETECTOR_ERROR_PIXEL_BY_SLOPE = 2.;
namespace rm_auto_aim {
/////////////////////////////////////important todo：在同时看到两块装甲板的时候，同时优化两块装甲板的姿态 参考上交开源
/////////！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
AngleOptimizer::AngleOptimizer(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
      // Unit: m
  constexpr double small_half_y = SMALL_ARMOR_WIDTH   / 2.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT  / 2.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH   / 2.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT  / 2.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}    

double AngleOptimizer::get_abs_angle(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    double dot = v1.dot(v2); // 点积
    double cross = v1.x() * v2.y() - v1.y() * v2.x(); // 叉积
    return std::abs(std::atan2(cross, dot));
}

double AngleOptimizer::sq(double x) {
    return x * x;
}

double AngleOptimizer::get_pts_cost(
    const std::vector<cv::Point2f>& cv_refs,
    const std::vector<cv::Point2f>& cv_pts,
    const double& inclined
) {
    std::size_t size = 4;//todo:这里不应该这样 应该是refs.size() 因为以后还要把两块装甲板同时优化
    std::vector<Eigen::Vector2d> refs;
    std::vector<Eigen::Vector2d> pts;
    for (std::size_t i = 0u; i < size; ++i) {
        refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
        pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
    }
    double cost = 0.;
    for (std::size_t i = 0u; i < size; ++i) {
        std::size_t p = (i + 1u) % size;
        // i - p 构成线段。过程：先移动起点，再补长度，再旋转
        Eigen::Vector2d ref_d = refs[p] - refs[i]; // 标准
        Eigen::Vector2d pt_d = pts[p] - pts[i];
        // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
        double pixel_dis = // dis 是指方差平面内到原点的距离
            (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm())
             + std::fabs(ref_d.norm() - pt_d.norm()))
            / ref_d.norm();
            // std::cout<<"pixel:"<<pixel_dis<<std::endl;
        double angular_dis = ref_d.norm() * get_abs_angle(ref_d, pt_d) / ref_d.norm();
        // std::cout<<"angul:"<<angular_dis<<std::endl;
        // 平方可能是为了配合 sin 和 cos
        // 弧度差代价（0 度左右占比应该大）
        double cost_i = sq(pixel_dis * std::sin(inclined))
            + sq(angular_dis * std::cos(inclined)) * DETECTOR_ERROR_PIXEL_BY_SLOPE;
        // 重投影像素误差越大，越相信斜率
        cost += std::sqrt(cost_i);
    }
    return cost;
}


//世界坐标系中装甲板四个角点的坐标
std::vector<cv::Point3f> AngleOptimizer::radial_armor_corners(
    const Armor armor
) {
    // std::vector<Eigen::Vector3d> corners;
    // image_armor_points.clear();
    std::vector<cv::Point3f> world_points;//todo:根据大小装甲板不同 设置不同的世界坐标 armor结构体里有armortype，写个if就行
    // // Fill in image points
    std::vector<cv::Point2f> image_armor_points_;
    image_armor_points_.emplace_back(armor.left_light.bottom);
    image_armor_points_.emplace_back(armor.left_light.top);
    image_armor_points_.emplace_back(armor.right_light.top);
    image_armor_points_.emplace_back(armor.right_light.bottom);
    image_armor_points = image_armor_points_;
    //   // Unit: m
    // constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 ;
    // constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 ;
    // // Start from bottom left in clockwise order
    // // Model coordinate: x forward, y left, z up
    // world_points.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
    // world_points.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
    // world_points.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
    // world_points.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

    auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;

    return object_points;
}
std::vector<cv::Point2f> AngleOptimizer::radial_armor_pts(
    std::vector<cv::Point3f> world_points,
    const cv::Mat &tvec,
    const cv::Mat &rvec
){
    std::vector<cv::Point2f> image_points;
    // 将世界坐标系中的点投影到图像平面//todo：建议仔细学习一下这个函数怎么用的

    cv::projectPoints(world_points, rvec, tvec, camera_matrix_, dist_coeffs_, image_points);
    return image_points;
}


geometry_msgs::msg::PoseStamped AngleOptimizer::odem_to_cameraopf(
    const std::shared_ptr<tf2_ros::Buffer> &tf2_buffer_,
    geometry_msgs::msg::PoseStamped &transformed_pose,
    double &yaw,
    double &pitch,
    cv::Mat &rvec
){
    // yaw = -9 * CV_PI / 180.0;
    pitch = 15* CV_PI / 180.0;//todo:如果外面传进来的是对的就把这里的删掉
    tf2::Quaternion fit_q;
    fit_q.setRPY(0, pitch, yaw); // roll保持不变，pitch=0，yaw=15度
    transformed_pose.pose.orientation = tf2::toMsg(fit_q);
    geometry_msgs::msg::PoseStamped camera_pose;
    tf2::Quaternion camera_fit_q;
    try {
        camera_pose = tf2_buffer_->transform(transformed_pose, "camera_optical_frame"); 
        tf2::fromMsg(camera_pose.pose.orientation, camera_fit_q);
    } catch (const tf2::TransformException & ex) {
        // RCLCPP_ERROR(get_logger(), "Error transforming pose to odom frame: %s", ex.what());
        // return; 
        std::cout<< ex.what()<<std::endl;//建议把这个rclcpp改回来
    }
    tf2::Matrix3x3 camera_new_rotation_matrix;
    camera_new_rotation_matrix.setRotation(camera_fit_q);

    cv::Mat new_rot_mat = cv::Mat::eye(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            new_rot_mat.at<double>(i, j) = camera_new_rotation_matrix[i][j];
        }
    }
    // 将新的旋转矩阵转换为旋转向量rvec
    cv::Mat new_rvec;
    cv::Rodrigues(new_rot_mat, new_rvec);
    rvec = new_rvec;
    
    return camera_pose;//todo 其实这里不用返回这玩意
}
double AngleOptimizer::cal_cost(
                  const Armor armor,
                  cv::Mat tvec,
                  cv::Mat rvec,
                  const std::vector<cv::Point3f> &world_points,
                  const std::shared_ptr<tf2_ros::Buffer> &tf2_buffer_,
                  geometry_msgs::msg::PoseStamped &transformed_pose,
                  double& x) 
{
    double pitch = 0.2618;

    if (armor.number == "outpost") 
    {
        pitch = -0.2618;// 这个pitch看下面的todo 改完下面的 删掉这一行
    }
    else
    {
        pitch = 0.2618;
    }
    // x = -0.17;
    // std::vector<cv::Point3f> world_points = radial_armor_corners(armor);
    odem_to_cameraopf(tf2_buffer_,transformed_pose,x,pitch,rvec);
    //todo:pitch角度写到armor里 根据装甲板类型区分        ↑这个
    // std::cout<<tvec<<std::endl;
    std::vector<cv::Point2f> pts = radial_armor_pts(world_points, tvec, rvec);
    image_pts = pts;
    // test1 = 1111;
    return get_pts_cost(image_armor_points, pts, x);
}
double AngleOptimizer::ternary_search_yaw(
        const Armor armor,
        cv::Mat tvec,
        cv::Mat rvec,
        const std::vector<cv::Point3f> &world_points,
        const std::shared_ptr<tf2_ros::Buffer> &tf2_buffer_,
        geometry_msgs::msg::PoseStamped &transformed_pose
) {
    // int iterations = 20;
    // double mid1, mid2;
    // double cost1, cost2;
    double low = -180* CV_PI / 180.0;
    double high = 180* CV_PI / 180.0;
    // for (int i = 0; i < iterations; ++i) {
    //     // 计算两个中间点
    //     mid1 = low + (high - low) / 3.0;
    //     mid2 = high - (high - low) / 3.0;
    //     // 计算两个中间点的代价
    //     cost1 = cal_cost(armor,tvec,rvec,world_points,tf2_buffer_,transformed_pose, mid1);
    //     cost2 = cal_cost(armor,tvec,rvec,world_points,tf2_buffer_,transformed_pose, mid2);
    //     std::cout<<mid1<<std::endl;
    //     std::cout<<mid2<<std::endl;
    //     std::cout<<std::endl;
    //     // 根据代价更新搜索范围
    //     if (cost1 < cost2) {
    //         low = mid1; // 如果 mid1 的代价更小，则更新左边界
    //     } else {
    //         high = mid2; // 如果 mid2 的代价更小，则更新右边界
    //     }
    //     std::cout<<"   "<<(low + high) / 2.0<<std::endl;
    // }
    // // 返回最优的 yaw 值
    // std::cout<<"1111111111111111111111"<<std::endl;
    // return (low + high) / 2.0;
    // // for(double i=-3.14;i<3.14;i+=0.5){
    // //     double cost = cal_cost(armor,tvec,rvec,world_points,tf2_buffer_,transformed_pose, i);
    // //     std::cout<<i<<"::"<<cost<<std::endl;
    // // }
    // // return 0.1;
    //**********上面是一个不能用的从天大抄来的奇奇怪怪的求解。 */
    double L = low, R = high; // 区间 [L, R]
    double M1, M2; // 黄金分割点
    double V1, V2; // 函数值

    // 初始化黄金分割点
    M1 = L + (R - L) * (3 - sqrt(5)) / 2; // 0.382...
    M2 = L + (R - L) * (sqrt(5) - 1) / 2; // 0.618...

    // VL = cal_cost(armor,tvec,rvec,world_points,tf2_buffer_,transformed_pose, L);;
    V1 = cal_cost(armor,tvec,rvec,world_points,tf2_buffer_,transformed_pose, M1);;
    V2 = cal_cost(armor,tvec,rvec,world_points,tf2_buffer_,transformed_pose, M2);;
    // VR = cal_cost(armor,tvec,rvec,world_points,tf2_buffer_,transformed_pose, R);;
    int cnt = 0;
    // 黄金分割搜索
    while (cnt<FIND_ANGLE_ITERATIONS) {
        if (V1 < V2) {
            // 最小值在 [L, M2] 区间
            R = M2;
            M2 = M1;
            V2 = V1;
            M1 = L + (R - L) * (3 - sqrt(5)) / 2;
            V1 = cal_cost(armor,tvec,rvec,world_points,tf2_buffer_,transformed_pose, M1);;
        } else {
            // 最小值在 [M1, R] 区间
            L = M1;
            M1 = M2;
            V1 = V2;
            M2 = L + (R - L) * (sqrt(5) - 1) / 2;
            V2 = cal_cost(armor,tvec,rvec,world_points,tf2_buffer_,transformed_pose, M2);
        }
        cnt++;
    }

    // 返回区间中点作为最小值点
    return (L + R) / 2;
}

}