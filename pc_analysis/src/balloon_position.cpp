/**
 * Used for mbzirc. Color detection of the balloon.
 * Will use YOLO for detection in the future.
 * Yongming 
 * 2020/01/04
 */

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>         // Eigen::Vector3d

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/centroid.h>      // computer3Dcentroid()


std::vector<double> thresholds;
pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZHSV>());

void callback_balloon(const sensor_msgs::PointCloud2ConstPtr &pc_in)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromROSMsg(*pc_in, *cloud_rgb_in);
  ROS_INFO("Size of cloud_rgb_in: %zu", cloud_rgb_in->points.size());

  // rgb to hsv
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv_in(new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::PointCloudXYZRGBAtoXYZHSV(*cloud_rgb_in, *cloud_hsv_in);
  ROS_INFO("Size of cloud_hsv_in: %zu", cloud_hsv_in->points.size());

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZHSV> cond_rem;
  cond_rem.setInputCloud(cloud_hsv_in);
  cond_rem.setCondition(range_cond);

  // apply filter
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZHSV>);
  cond_rem.filter(*cloud_color);
  ROS_INFO("Size of cloud_color: %zu", cloud_color->points.size());

  pcl::RadiusOutlierRemoval<pcl::PointXYZHSV> out_rem;
  // build the filter
  out_rem.setInputCloud(cloud_color);
  out_rem.setRadiusSearch(0.45);
  out_rem.setMinNeighborsInRadius(20);
  // apply filter
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color_radius(new pcl::PointCloud<pcl::PointXYZHSV>);
  out_rem.filter(*cloud_color_radius);
  ROS_INFO("Size of cloud_color_radius: %zu", cloud_color_radius->points.size());

  Eigen::Vector4d centroid;
  pcl::compute3DCentroid (*cloud_color_radius, centroid);
  ROS_INFO("centroid: %f, %f, %f, %f", centroid[0], centroid[1], centroid[2], centroid[3]);
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "balloon_position");
  if (argc == 7) {
    for (int i = 0; i < 6; ++i) {thresholds.push_back( std::stod(argv[i+1]) );} 
  } else if (argc == 1) {
    thresholds = {0, 180, 100, 256, 100, 256}; // Red
  } else {
    ROS_INFO("Wrong argument number!");
    return -1;
  }

  ROS_INFO("%f %f %f %f %f %f", thresholds[0],thresholds[1],thresholds[2],thresholds[3],thresholds[4],thresholds[5]);

  // build the filter condition
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GT, double( thresholds[0]*2) )));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LT, double(thresholds[1]*2) )));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GT, double(thresholds[2] / 255) )));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LT, double(thresholds[3] / 255) )));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GT, double(thresholds[4] / 255) )));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LT, double(thresholds[5] / 255) )));

  ros::NodeHandle nh("~");
  std::string topic_pointcloud;
  nh.param<std::string>("topic_pointcloud", topic_pointcloud, "/zed/point_cloud/cloud_registered");
  ROS_INFO_STREAM(topic_pointcloud);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic_pointcloud, 1, callback_balloon);
  
  // Subscribe for a period
  ros::Time t;
  t = t.now();
  ros::Rate loop_rate(2);
  while (ros::ok() && (t.now() -t).toSec() <= 3600)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}