#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/features/normal_3d.h>
#include <vector>
#include <ctime>
#include <chrono>
template <class PointX>
class PointProcessor
{
public:

      typename pcl::PointCloud<PointX>::Ptr FilterCloud(
            typename pcl::PointCloud<PointX>::Ptr cloud,
            float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
    {
        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();
        typename pcl::PointCloud<PointX>::Ptr cloud_filtered(new pcl::PointCloud<PointX>());
        typename pcl::PointCloud<PointX>::Ptr cloud_roi(new pcl::PointCloud<PointX>());
        // Convert the points to voxel grid points
        pcl::VoxelGrid<PointX> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (filterRes, filterRes, filterRes);
        sor.filter (*cloud_filtered);
        std::cerr << "Voxeled size  " << cloud_filtered->points.size () << std::endl;
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

//        pcl::CropBox<PointX> cropBoxFilter (true);
//        cropBoxFilter.setInputCloud (cloud_filtered);
//        Eigen::Vector4f min_pt (-4.0f, -0.4f, 1.0f, 1.0f);
//        Eigen::Vector4f max_pt (4.0f, 0.9f, 14.0f, 1.0f);
        Eigen::Vector4f min_pt (-4.0f, 0.41f, 1.0f, 1.0f);
        Eigen::Vector4f max_pt (4.0f, 0.5f, 14.0f, 1.0f);

//        // Cropbox slighlty bigger then bounding box of points
//        cropBoxFilter.setMin (min_pt);
//        cropBoxFilter.setMax (max_pt);

//        // Indices
//        std::vector<int> indices;
//        cropBoxFilter.filter (indices);

//        // Cloud
//        cropBoxFilter.filter (*cloud_roi);

          std::cerr << "$$$$$$$$$$$$$$$$  ROI size  " << cloud_roi->points.size () << std::endl;


            return cloud_filtered;
    }

    typename pcl::PointCloud<PointX>::Ptr SegmentGroundPlane(
            typename pcl::PointCloud<PointX>::Ptr cloud,
            int maxIterations, float distanceThreshold)
    {
        std::cout << "original cloud contains " << cloud->points.size() << " Points" << std::endl;
        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();
            pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
            // Create the segmentation object
            pcl::SACSegmentation<PointX> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (maxIterations);
            seg.setDistanceThreshold (distanceThreshold);

            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
              std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            }
           std::pair<typename pcl::PointCloud<PointX>::Ptr, typename pcl::PointCloud<PointX>::Ptr> segResult = SeparateClouds(inliers,cloud);
           auto endTime = std::chrono::steady_clock::now();
           auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
           std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
           std::cout << "plane segmentation contains " << segResult.second->points.size() << " Points" << std::endl;
           return segResult.first;
    }

    std::vector<typename pcl::PointCloud<PointX>::Ptr> cluster(typename pcl::PointCloud<PointX>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
    {
        // Time clustering process
        auto startTime = std::chrono::steady_clock::now();

        std::vector<typename pcl::PointCloud<PointX>::Ptr> clusters;

            typename pcl::search::KdTree<PointX>::Ptr tree (new pcl::search::KdTree<PointX>);
            tree->setInputCloud (cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<PointX> ec;
            ec.setClusterTolerance (clusterTolerance); // 2cm
            ec.setMinClusterSize (minSize);
            ec.setMaxClusterSize (maxSize);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud);
            ec.extract (cluster_indices);

            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
              {
                    typename pcl::PointCloud<PointX>::Ptr cloud_cluster (new pcl::PointCloud<PointX>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                  cloud_cluster->points.push_back (cloud->points[*pit]); //*
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                clusters.push_back(cloud_cluster);
              }
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
        // Find bounding box for one of the clusters
//        PointX minPoint, maxPoint;
//        std::vector<PointX>
//        for(pcl::PointCloud<PointX>::Ptr cluster : clusters)
//           {
//            pcl::getMinMax3D(*cluster, minPoint, maxPoint);


//            }

        return clusters;
    }
    // ----------------------------------------------------------------
    // -----Calculate surface normals with a search radius of 0.05-----
    // ----------------------------------------------------------------
    void calcSurfaceNormals(typename pcl::PointCloud<PointX>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
      typename pcl::NormalEstimation<PointX, pcl::Normal> ne;
      ne.setInputCloud(cloud);
      typename pcl::search::KdTree<PointX>::Ptr tree(new pcl::search::KdTree<PointX>());
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(0.05);
      ne.compute(*normals);
    }

    // ---------------------------------------
    // -----Initialize Occupancy Grid Msg-----
    // ---------------------------------------
    void initGrid(nav_msgs::msg::OccupancyGrid::Ptr grid) {
      // grid->header.seq = 1;
      grid->header.frame_id = "world";
      grid->info.origin.position.z = 0;
      grid->info.origin.orientation.w = 1;
      grid->info.origin.orientation.x = 0;
      grid->info.origin.orientation.y = 0;
      grid->info.origin.orientation.z = 0;
    }

    // -----------------------------------
    // -----Update Occupancy Grid Msg-----
    // -----------------------------------
    void updateGrid(nav_msgs::msg::OccupancyGrid::Ptr grid, double cellRes, int xCells, int yCells,
        double originX, double originY, std::vector<signed char> *ocGrid) {
//      grid->header.seq++;
//      grid->header.stamp.sec = ros::Time::now().sec;
//      grid->header.stamp.nsec = ros::Time::now().nsec;
//      grid->info.map_load_time = ros::Time::now();
//      grid->info.resolution = cellRes;
//      grid->info.width = xCells;
//      grid->info.height = yCells;
//      grid->info.origin.position.x = originX;
//      grid->info.origin.position.y = originY;
      grid->data = *ocGrid;
    }

    // ------------------------------------------
    // -----Calculate size of Occupancy Grid-----
    // ------------------------------------------
    void calcSize(typename pcl::PointCloud<PointX>::Ptr cloud, double *xMax, double *yMax, double *xMin, double *yMin) {
      for (size_t i = 0; i < cloud->size(); i++) {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;
        if (*xMax < x) {
          *xMax = x;
        }
        if (*xMin > x) {
          *xMin = x;
        }
        if (*yMax < y) {
          *yMax = y;
        }
        if (*yMin > y) {
          *yMin = y;
        }
      }
    }

    // ---------------------------------------
    // -----Populate map with cost values-----
    // ---------------------------------------
    void populateMap(typename pcl::PointCloud<PointX>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::vector<int> &map, double xMin, double yMin,
        double cellResolution, int xCells, int yCells) {
      double deviation = 0.78539816339;

      for (size_t i = 0; i < cloud->size(); i++) {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;
        double z = cloud->points[i].normal_z;

        double phi = acos(fabs(z));
        int xCell, yCell;

        if (z == z) { //TODO implement cutoff height
          xCell = (int) ((x - xMin) / cellResolution);
          yCell = (int) ((y - yMin) / cellResolution);
          if ((yCell * xCells + xCell) > (xCells * yCells)) {
            std::cout << "x: " << x << ", y: " << y << ", xCell: " << xCell << ", yCell: " << yCell
                << "\n";
          }
          if (phi > deviation) {
            map[yCell * xCells + xCell]++;
          } else {
            map[yCell * xCells + xCell]--;
          }
        }
      }
    }

    // ---------------------------------
    // -----Generate Occupancy Grid-----
    // ---------------------------------
    void genOccupancyGrid(std::vector<signed char> &ocGrid, std::vector<int> &countGrid, int size) {
      int buf = 5;
      for (int i = 0; i < size; i++) {
        if (countGrid[i] < buf) {
          ocGrid[i] = 0;
        } else if (countGrid[i] > buf) {
          ocGrid[i] = 100;
        } else if (countGrid[i] == 0) {
          ocGrid[i] = 0; // TODO Should be -1
        }
      }
    }




private:

    std::pair<typename pcl::PointCloud<PointX>::Ptr, typename pcl::PointCloud<PointX>::Ptr>
    SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointX>::Ptr cloud)
    {
            typename pcl::PointCloud<PointX>::Ptr obstCloud(new pcl::PointCloud<PointX>());
            typename pcl::PointCloud<PointX>::Ptr planeCloud(new pcl::PointCloud<PointX>());

            for(int it : inliers->indices)
            {
                    planeCloud->points.push_back(cloud->points[it]);
            }

            pcl::ExtractIndices<PointX> extract;
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*obstCloud);

        std::pair<typename pcl::PointCloud<PointX>::Ptr, typename pcl::PointCloud<PointX>::Ptr> segResult(obstCloud, planeCloud);
        return segResult;
    }


};
