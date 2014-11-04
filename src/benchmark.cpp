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

  //Variables de configuración: ángulo de agarre, distancias...
  double angle, rad, along;
  bool alignedGrasp;

  std::string input_basename("output");
  nh.getParam("input_basename", input_basename);
  nh.param("alignedGrasp", alignedGrasp, false);
  nh.getParam("angle", angle);
  nh.getParam("rad", rad);
  nh.getParam("along", along);

  //Point Cloud load
  std::string point_cloud_file(input_basename + std::string(".pcd"));
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read(point_cloud_file, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;


  bool i = 0;
  int iterations=500;

  while (ros::ok() && i<9)
  {
    //Init planner
    ROS_ERROR_STREAM("Iterations: "<<iterations);
    PCAutonomousGraspPlanning planner(angle, rad, along, alignedGrasp, cloud);
    planner.setPlaneSegmentationParams(0.03, 100);
    planner.setCylinderSegmentationParams(0.05, iterations, 0.1);
    planner.perceive();

    iterations-=70;
    i++;
  }
  return 0;
}
