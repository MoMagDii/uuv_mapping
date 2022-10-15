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
#define BOOST_BIND_NO_PLACEHOLDERS
#include "perception_node.hpp"
#include <math.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using std::placeholders::_1;

/* Constructor */
// =========================================================================
Perception::Perception(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
    cv::namedWindow(OPENCV_WINDOW_);
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  // Subscribers initialization
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/zed2/points_demo", 10, std::bind(&Perception::cloudCallback, this, _1));
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered/cloud", 10);
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/grid", 10);
  img_sub_ = image_transport::create_subscription(this, "/zed2/image_demo",
          std::bind(&Perception::imageCallback, this, std::placeholders::_1), "raw", custom_qos);
  img_pub_ = image_transport::create_publisher(this, "out_image_base_topic", custom_qos);
}
// =========================================================================
void Perception::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
     // This will convert the message into a pcl::PointCloud
    pcl::fromROSMsg(*msg, *cloud);
     auto filtered_cloud = pointProcessor_.FilterCloud(cloud, 0.04 , Eigen::Vector4f(0.0,-1.0,-1.0,1.0), Eigen::Vector4f(15.0,1.0,1.0,1.0));
     auto segment_cloud = pointProcessor_.SegmentGroundPlane(filtered_cloud,100, 0.01);
     // auto clusters = pointProcessor_.cluster(filtered_cloud, 0.2, 2, 20);
     /* Initialize Grid */
     nav_msgs::msg::OccupancyGrid::Ptr grid(new nav_msgs::msg::OccupancyGrid);
     pointProcessor_.initGrid(grid);
     //mutex.lock();
     //mutex.unlock();
     /* Calculate Surface Normals */
     pcl::PointCloud<pcl::Normal>::Ptr  cloud_normals(new pcl::PointCloud<pcl::Normal>);
     pointProcessor_.calcSurfaceNormals(filtered_cloud, cloud_normals);
     /* Figure out size of matrix needed to store data. */
     double xMax = 0, yMax = 0, xMin = 0, yMin = 0;
     pointProcessor_.calcSize(filtered_cloud, &xMax, &yMax, &xMin, &yMin);
     /* Determine resolution of grid (m/cell) */
     double cellResolution = 0.7;
     int xCells = ((int) ((xMax - xMin) / cellResolution)) + 1;
     int yCells = ((int) ((yMax - yMin) / cellResolution)) + 1;
     std::vector<int> countGrid(yCells * xCells);
    // pointProcessor_.populateMap(cloud_normals, countGrid, xMin, yMin, cellResolution, xCells, yCells);

     /* Generate OccupancyGrid Data Vector */
     std::vector<signed char> ocGrid(yCells * xCells);
     pointProcessor_.genOccupancyGrid(ocGrid, countGrid, xCells * yCells);

     /* Update OccupancyGrid Object */
     pointProcessor_.updateGrid(grid, cellResolution, xCells, yCells, xMin, yMin, &ocGrid);
     publish_cloud(segment_cloud);
}
//=========================================================================
void Perception::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    cv::Mat img = cv_ptr->image;
    // convert current image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image
    cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
    double t = (double)cv::getTickCount();
    detector->detect(imgGray, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "BRISK detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::imshow(OPENCV_WINDOW_, visImage);
    cv::waitKey(3);

    img_pub_.publish(cv_ptr->toImageMsg());

}
// =========================================================================
void Perception::publish_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "swift/camera_link_optical";
    pcl::toROSMsg(*cloud, msg);
    cloud_pub_->publish(msg);
}
// =========================================================================
inline Eigen::VectorXd Perception::vector_to_eigen(std::vector<double> v)
{
  // A method for converting std::vector<double> to Eigen::VectorXd
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}
// =========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments({"perception_node"});
  std::shared_ptr<Perception> node = std::make_shared<Perception>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
