// Copyright 2022 VorteX-co
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAJECTORY_TRACKING_LQR__CONTROLLER_NODE_HPP_
#define TRAJECTORY_TRACKING_LQR__CONTROLLER_NODE_HPP_

#include <memory>
#include <vector>
#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "image_transport/image_transport.hpp"
#include "point_processor.hpp"


class Perception: public rclcpp::Node
{
public:
  /**
   * @brief Constructor that initializes all the subscribes and publishers
   * @param Node options contains node name and paramteres from .yaml file
   */
  explicit Perception(const rclcpp::NodeOptions & options);

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;

  void publish_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
   void publish_cloud(nav_msgs::msg::OccupancyGrid::Ptr grid);
  inline Eigen::VectorXd vector_to_eigen(std::vector<double> v);

  PointProcessor<pcl::PointXYZ> pointProcessor_;
  const std::string OPENCV_WINDOW_ = "Image window";

};
#endif  // TRAJECTORY_TRACKING_LQR__CONTROLLER_NODE_HPP_
