#include "pcl_manipulation/arm5e_pc_grasp_planning_server.h"

#include <mar_perception/PCAutonomousGraspPlanning.h>
#include <unistd.h>

pcl_manipulation::ObjectDetectionServer::ObjectDetectionServer(
  ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  nh_(nh), nh_private_(nh_private)
{

  service_server_ = nh_private_.advertiseService("detect_object",
      &ObjectDetectionServer::serviceCallback, this);
  ROS_INFO("Service \"detect_object\" advertised.");
  cloud_found_=false;
}

bool pcl_manipulation::ObjectDetectionServer::serviceCallback(
    DetectObjectRequest& req,
    DetectObjectResponse& res)
{
  ROS_INFO("Object detector called. Starting detection.");
  //subscribe to pointcloud...
  sub_=nh_private_.subscribe("/stereo_down/points2", 5, &ObjectDetectionServer::cloudCallback, this); //Parameterize topic name
  //wait for subscriber & cloud...
  while(!cloud_found_){ROS_INFO("*");ros::spinOnce();sleep(1);}
  ROS_INFO("Cloud passed. Starting perception...");
  PCAutonomousGraspPlanning planner(req.angle, req.rad, req.along, true, cloud_);
  planner.setPlaneSegmentationParams(0.09, 200);
  planner.setCylinderSegmentationParams(0.06, 10000, 0.1);
  planner.perceive();
  ROS_INFO("Perception finished.");
  vpHomogeneousMatrix cMo = planner.get_cMg();
  geometry_msgs::Pose p;
  p.position.x=cMo[0][3];
  p.position.y=cMo[1][3];
  p.position.z=cMo[2][3];
  vpQuaternionVector q; cMo.extract(q);
  p.orientation.x=q.x();
  p.orientation.y=q.y();
  p.orientation.z=q.z();
  p.orientation.w=q.w();
  res.cMo=p;
  res.success=true;
  return true;
}


void pcl_manipulation::ObjectDetectionServer::cloudCallback(const sensor_msgs::PointCloud2 input)
{
  //ROS Points2 message to PCL.
  ROS_INFO_STREAM("Acquiring cloud.");
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(input, pcl_pc);
  //PCL Generic cloud to XYZRGB strong type.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_=cloud;
  pcl::fromPCLPointCloud2(pcl_pc, *cloud_);
  ROS_INFO_STREAM("Cloud size: " << cloud_->width * cloud_->height << "points.");
  //ONLINE FILTERING from save_cloud.cpp

  cloud_found_=true;
  sub_.shutdown();
}


int main(int argc, char **argv){
  ros::init(argc, argv, "object_detection_server");
  ros::NodeHandle nh;
  pcl_manipulation::ObjectDetectionServer ods(nh,nh);
  ros::spin();
  return 0;
}
