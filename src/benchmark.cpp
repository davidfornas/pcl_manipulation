/**
 *
 *  Created on: 3/11/2014
 *      Author: dfornas
 */
//MAR
#include <mar_perception/PCAutonomousGraspPlanning.h>
#include <mar_ros_bridge/mar_params.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ctime>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "segmentation_benchmark");
  ros::NodeHandle nh;

  std::string input_basename("output");
  nh.getParam("input_basename", input_basename);

  //Point Cloud load
  std::string point_cloud_file(input_basename + std::string(".pcd"));
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read(point_cloud_file, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;


  bool i = 0;
  int iterations=1, inc=1;

  while (ros::ok() && i<iterations)
  {
    //Init planner
    ROS_ERROR_STREAM("Iterations: " << iterations);
    PCAutonomousGraspPlanning planner(0, 0, 0, false, cloud);
    planner.setPlaneSegmentationParams(0.08, 200);
    //planner.setCylinderSegmentationParams(0.02+0.01*i, 5000, 0.1);
    planner.perceive();

    i++;
  }
  return 0;
}
