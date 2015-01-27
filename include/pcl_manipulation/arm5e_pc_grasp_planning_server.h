#ifndef GRASP_PLANNING_OBJECT_DETECTOR_SERVER
#define GRASP_PLANNING_OBJECT_DETECTOR_SERVER

#include "pcl_manipulation/DetectObject.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//PCL Includes
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace pcl_manipulation
{

class ObjectDetectionServer
{
public:

  ObjectDetectionServer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

private:

  bool serviceCallback(DetectObjectRequest& req, DetectObjectResponse& res);
  void cloudCallback(const sensor_msgs::PointCloud2 input);

  ros::NodeHandle nh_, nh_private_;
  ros::ServiceServer service_server_;

  ros::Subscriber sub_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;


};

} // end of namespace


#endif
