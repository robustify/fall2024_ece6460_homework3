// Header file for the class
#include "Homework3.hpp"

// Namespace matches ROS package name
namespace homework3 
{  
  // Constructor with global and private node handle arguments
  Homework3::Homework3(ros::NodeHandle& n, ros::NodeHandle& pn) :
    listener_(buffer_),
    kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
  {
    sub_cloud_ = n.subscribe<sensor_msgs::PointCloud2>("ouster/points_raw", 10, &Homework3::recvCloud, this);
    pub_merged_cluster_cloud_ = n.advertise<sensor_msgs::PointCloud2>("merged_cluster_cloud", 1);
    pub_bboxes_ = n.advertise<avs_lecture_msgs::TrackedObjectArray>("objects", 1);
    pub_normals_ = n.advertise<geometry_msgs::PoseArray>("normals", 1);
  }

  void Homework3::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Apply coordinate frame transformation
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("base_footprint", *msg, transformed_msg, buffer_);

    // Copy into PCL cloud for processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // TODO: Copy transformed_msg into input_cloud

    // Run the processing pipeline
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    passthroughFilter(input_cloud, filtered_cloud);
    voxelFilter(filtered_cloud, filtered_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    normalsFilter(filtered_cloud, no_ground_cloud);

    // TODO: Return from function here if there are no points left after filtering out the ground points

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    euclideanClustering(no_ground_cloud, cluster_clouds);
    generateBoundingBoxes(cluster_clouds);
    bboxes_.header = pcl_conversions::fromPCL(no_ground_cloud->header);

    mergeClusters(cluster_clouds);
    merged_cluster_cloud_.header = pcl_conversions::fromPCL(no_ground_cloud->header);

    pub_merged_cluster_cloud_.publish(merged_cluster_cloud_);
    pub_bboxes_.publish(bboxes_);
    pub_normals_.publish(normals_);
  }

  void Homework3::passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // TODO: Implement passthrough filter here. Put final output in the 'cloud_out' argument
  }

  void Homework3::voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // TODO: Implement voxel downsampling filter here. Put final output in the 'cloud_out' argument
  }

  void Homework3::normalsFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // Compute normal vectors for the incoming point cloud
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    kd_tree_->setInputCloud(cloud_in);
    normal_estimator.setSearchMethod(kd_tree_);
    normal_estimator.setInputCloud(cloud_in);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*cloud_normals);

    // Filter out near-vertical normals
    pcl::PointIndices non_vertical_normals;
    for (int i = 0; i < cloud_normals->points.size(); i++) {
      // TODO: Compute the angle from vertical for the current normal vector
      //       and if it is greater than 30 degrees from vertical, add the index
      //       to the 'non_vertical_normals' array
      non_vertical_normals.indices.push_back(i);
    }
    pcl::copyPointCloud(*cloud_in, non_vertical_normals, *cloud_out);

    // Populate PoseArray message to visualize normals
    normals_.header = pcl_conversions::fromPCL(cloud_in->header);
    normals_.poses.clear();
    for (int i = 0; i < non_vertical_normals.indices.size(); i++) {
      geometry_msgs::Pose p;
      p.position.x = cloud_in->points[non_vertical_normals.indices[i]].x;
      p.position.y = cloud_in->points[non_vertical_normals.indices[i]].y;
      p.position.z = cloud_in->points[non_vertical_normals.indices[i]].z;

      double nx = cloud_normals->points[non_vertical_normals.indices[i]].normal_x;
      double ny = cloud_normals->points[non_vertical_normals.indices[i]].normal_y;
      double nz = cloud_normals->points[non_vertical_normals.indices[i]].normal_z;

      // Construct rotation matrix to align frame transform with the normal vector
      tf2::Matrix3x3 rot_mat;
      // First basis vector is the vector we want to align
      rot_mat[0] = tf2::Vector3(nx, ny, nz);
      if (std::abs(nz) < 0.9) {
        // Vector is not close to vertical --> use x and y components to create orthogonal vector
        rot_mat[1] = tf2::Vector3(-ny, nx, 0);
      } else {
        // Vector is close to vertical --> use y and z components to make orthogonal vector
        rot_mat[1] = tf2::Vector3(0, -nz, ny);
      }
      // Normalize the generated orthogonal vector, because it is not necessarily unit length
      rot_mat[1].normalize();
      // Cross product produces the third basis vector of the rotation matrix
      rot_mat[2] = rot_mat[0].cross(rot_mat[1]);

      // Extract equivalent quaternion representation for the transform
      // rot_mat.transpose() is used because the basis vectors should be loaded
      // into the columns of the matrix, but the indexing in the above commands set the rows
      //   of the matrix instead of the columns.
      tf2::Quaternion q;
      rot_mat.transpose().getRotation(q);

      // Fill orientation of pose structure
      tf2::convert(q, p.orientation);
      normals_.poses.push_back(p);
    }
  }

  void Homework3::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    // TODO: Implement Euclidean clustering here, dumping the array of separated clouds into the 'cluster_clouds' argument
  }

  void Homework3::mergeClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    // TODO: Loop through the 'cluster_clouds' argument and merge all the points into a single PCL
    //       point cloud. Then copy the merged cloud into the 'merged_cluster_cloud_' message
  }

  void Homework3::generateBoundingBoxes(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    pcl::PointXYZ min_point, max_point;
    bboxes_.objects.clear();
    int bbox_id = 0;
    for (auto& cluster : cluster_clouds) {
      pcl::getMinMax3D(*cluster, min_point, max_point);
      avs_lecture_msgs::TrackedObject box;
      box.header = bboxes_.header;
      // TODO: Fill in the rest of the 'box' variable
      //         - Set `spawn_time` to the current ROS time
      //         - Increment the 'id' field for each cluster
      //         - Populate 'pose.position' with the midpoint between min_point and max_point
      //         - Populate 'pose.orientation' with an identity quaternion
      //         - Leave 'velocity.linear' and 'velocity.angular' unpopulated
      //         - Populate 'bounding_box_scale' with data from min_point and max_point
      //         - Leave 'bounding_box_offset' unpopulated
      bboxes_.objects.push_back(box);
    }
  }

}
