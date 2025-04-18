// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <string>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // Detector
  detector_ = initDetector();

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
  
    // 初始化 tf2_buffer_
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

  // 设置 tf2_buffer_ 的定时器接口
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  target_frame_ = this->declare_parameter("target_frame", "base_link");
  // 初始化 tf2_listener_
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // Aim mode param change moniter
  aim_mode_ = static_cast<AimMode>(this->declare_parameter("aim_mode", 0));

  // Debug Publishers
  debug_ = this->declare_parameter("debug", false);
  if (debug_) {
    createDebugPublishers();
  }

  // 获得aim_mode数据
  mode_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  mode_cb_handle_ =
    mode_param_sub_->add_parameter_callback("aim_mode", [this](const rclcpp::Parameter & p) {
      aim_mode_ = static_cast<AimMode>(p.as_int());
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      angle_optimizer_ = std::make_unique<AngleOptimizer>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  aim_mode_ = 0;
  // detector_->unrecognized_num=0;
  // std::cout << "detector_color            "<< detector_->detect_color <<std::endl;

    auto armors = detectArmors(img_msg);
    if (pnp_solver_ != nullptr) {
      armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
      armors_msg_.armors.clear();
      marker_array_.markers.clear();
      armor_marker_.id = 0;
      text_marker_.id = 0;

      auto_aim_interfaces::msg::Armor armor_msg;
      for (const auto & armor : armors) {
        cv::Mat rvec, tvec;
        bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
        if (success) {
          // Fill basic info
          armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
          armor_msg.number = armor.number;

          // Fill pose
          armor_msg.pose.position.x = tvec.at<double>(0);
          armor_msg.pose.position.y = tvec.at<double>(1);
          armor_msg.pose.position.z = tvec.at<double>(2);
          // rvec to 3x3 rotation matrix
          cv::Mat rotation_matrix;
          cv::Rodrigues(rvec, rotation_matrix);
          // rotation matrix to quaternion
          tf2::Matrix3x3 tf2_rotation_matrix(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
            rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
            rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
            rotation_matrix.at<double>(2, 2));
          tf2::Quaternion tf2_q;
          tf2_rotation_matrix.getRotation(tf2_q);
          armor_msg.pose.orientation = tf2::toMsg(tf2_q);
          
          /************************************************ The part of three-point Method ************************************************/

          double yaw, pitch, roll;
          tf2_rotation_matrix.getRPY(roll, pitch, yaw);

          /************************************************ The part of three-point Method ************************************************/


          // std::cout<<yaw*180/CV_PI<<"  "<<pitch*180/CV_PI<<"  "<<roll*180/CV_PI<<std::endl;
          // Eigen::Vector3d pos_(yaw, pitch, roll);
              // 设置新的yaw和pitch（15度和0度，转换为弧度）
              // std::cout<<"111";
          //////////////////////////  
          // geometry_msgs::msg::PoseStamped transformed_pose;
          // transformed_pose.header = armors_msg_.header;
          // transformed_pose.pose = armor_msg.pose;

          cv::Mat new_rvec;
          auto world_points_ = angle_optimizer_->radial_armor_corners(armor);

          //使用新的欧拉角创建旋转矩阵
          geometry_msgs::msg::PoseStamped pose_stamped;
          geometry_msgs::msg::PoseStamped transformed_pose;
          tf2::Matrix3x3 new_rotation_matrix;
          pose_stamped.header = armors_msg_.header;  // 设置当前坐标系
          pose_stamped.pose = armor_msg.pose;

          try {
            transformed_pose = tf2_buffer_->transform(pose_stamped, target_frame_);  
          } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(get_logger(), "Error transforming pose to odom frame: %s", ex.what());
            return; 
          }

          //std::cout<<"     "<<transformed_pose.pose.position.x<<"  " <<transformed_pose.pose.position.y<<"  " <<transformed_pose.pose.position.z<<"  " <<std::endl;

          double fit_yaw = angle_optimizer_->ternary_search_yaw(
            armor,
            tvec,
            rvec,
            world_points_,
            tf2_buffer_,
            transformed_pose);
            std::cout<<"11  " <<fit_yaw<<std::endl;

            armor_msg.fit_yaw = fit_yaw;


          // Fill the distance to image center
          armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

          // Fill the markers
          armor_marker_.id++;
          armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
          armor_marker_.pose = armor_msg.pose;
          text_marker_.id++;
          text_marker_.pose.position = armor_msg.pose.position;
          text_marker_.pose.position.y -= 0.1;
          text_marker_.text = armor.classfication_result;
          armors_msg_.armors.emplace_back(armor_msg);
          marker_array_.markers.emplace_back(armor_marker_);
          marker_array_.markers.emplace_back(text_marker_);
        } else {
          RCLCPP_WARN(this->get_logger(), "PnP failed!");
        }
      }

      // Publishing detected armors
      armors_pub_->publish(armors_msg_);

      // Publishing marker
      publishMarkers();
    }
  
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

  param_desc.description="0-negative  1:7-unrecognized_num";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 75431;
  int unrecognized_num = declare_parameter("unrecognized_num", 0, param_desc);

  // int unrecognized_num=this->declare_parameter("unrecognized_num",0);

  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.5),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color,unrecognized_num, l_params, a_params);
  
  // Init classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  // std::vector<std::string> ignore_classes =
  //   this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
  
   std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});

  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");
  // RCLCPP_INFO_STREAM(this->get_logger(), "Latency: " << latency << "ms");
  // Convert ROS img to cv::Mat

  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // Update params
  detector_->binary_thres = get_parameter("binary_thres").as_int();
    // std::cout<<"unrecognized_num-----   "<<detector_->detect_color<<std::endl;

  detector_->detect_color = get_parameter("detect_color").as_int();
    // std::cout<<"detect_color : "<<detector_->detect_color<<std::endl;

  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  detector_->unrecognized_num=get_parameter("unrecognized_num").as_int();
//  detector_->unrecognized_num=0;
  //  detector_->detect_color = 1;

  // std::cout<<"unrecognized_num   "<<detector_->detect_color<<std::endl;
  detector_->classifier->unrecognized_num_[0]=std::to_string(detector_->unrecognized_num%10);
  detector_->classifier->unrecognized_num_[1]=std::to_string(detector_->unrecognized_num/1000%10);
  detector_->classifier->unrecognized_num_[2]=std::to_string(detector_->unrecognized_num/100%10);
  detector_->classifier->unrecognized_num_[3]=std::to_string(detector_->unrecognized_num/10%10);
  // detector_->classifier->unrecognized_num_[4]=std::to_string(detector_->unrecognized_num%10);

  //printf("unrecognized_num_   %d\n",detector_->unrecognized_num);
  if(detector_->unrecognized_num%10000==7)
  {
    detector_->classifier->unrecognized_num_[4]="guard";
  }
  else
  {
    detector_->classifier->unrecognized_num_[4]="0";

  }
  // int tmp=detector_->unrecognized_num/10;
  // for(int i=0;i<4;++i)
  // {
  //   if(tmp%10!=0)
  //   {
  //     detector_->classifier->ignore_classes_.push_back(std::to_string(tmp%10));
  //   }
    
  //   tmp/=10;
  // }
  // std::cout<<"sssssss"<<detector_->classifier->ignore_classes_.size()<<std::endl;
  // detector_->classifier->ignore_classes_.push_back(std::to_string(detector_->unrecognized_num/10000));
  // detector_->classifier->ignore_classes_.push_back(std::to_string(detector_->unrecognized_num/1000%10));
  // detector_->classifier->ignore_classes_.push_back(std::to_string(detector_->unrecognized_num/100%10));
  // detector_->classifier->ignore_classes_.push_back(std::to_string(detector_->unrecognized_num/10%10));
  // detector_->classifier->ignore_classes_.push_back(std::to_string(detector_->unrecognized_num%10));



  //red is 0      blue is 1
  detector_->detect_color = 1; 
  std::cout<<"detector_color        : "<<detector_->detect_color<<std::endl;
  auto armors = detector_->detect(img);



  // Publish debug info
  if (debug_) {
    binary_img_pub_.publish(
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty()) {
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(
        *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }

    detector_->drawResults(img);
    
    for (const auto& point :  angle_optimizer_->image_pts) {
      cv::circle(img, point, 5, cv::Scalar(0, 255, 100), -1);  // 绘制绿色圆点
    }
    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

void ArmorDetectorNode::createDebugPublishers()
{
  // Init armors debug publishers
    lights_data_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
    armors_data_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

    binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
    number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
    result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
  

}

void ArmorDetectorNode::destroyDebugPublishers()
{
    lights_data_pub_.reset();
    armors_data_pub_.reset();

    binary_img_pub_.shutdown();
    number_img_pub_.shutdown();
    result_img_pub_.shutdown();
  
}

void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
