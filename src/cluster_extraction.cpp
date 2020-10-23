#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include "visualization_msgs/Marker.h"

ros::Publisher pub;
ros::Publisher marker_pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL data type
  // pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  // Perform the actual filtering
// Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->size ();
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
    
    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
    
    std::cout << "Max x: " << maxPt.x() << std::endl;
    std::cout << "Max y: " << maxPt.y() << std::endl;
    std::cout << "Max z: " << maxPt.z() << std::endl;
    std::cout << "Min x: " << minPt.x() << std::endl;
    std::cout << "Min y: " << minPt.y() << std::endl;
    std::cout << "Min z: " << minPt.z() << std::endl;
    
    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = j;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.pose.position.x = (maxPt.x() + minPt.x()) / 2;
    object_marker.pose.position.y = (maxPt.y() + minPt.y()) / 2;
    object_marker.pose.position.z = (maxPt.z() + minPt.z()) / 2;
    object_marker.pose.orientation.w = 1;
    object_marker.scale.x = maxPt.x() - minPt.x();
    object_marker.scale.y = maxPt.y() - minPt.y();
    object_marker.scale.z = maxPt.z() - minPt.z();
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    marker_pub.publish(object_marker);
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 pcl_pc2_2;
  pcl::toPCLPointCloud2(*cloud_filtered, pcl_pc2_2);
  pcl_conversions::fromPCL(pcl_pc2_2, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud_pcd", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker> ("box", 1);

  // Spin
  ros::spin ();
}
