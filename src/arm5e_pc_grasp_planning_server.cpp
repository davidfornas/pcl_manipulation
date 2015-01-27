#include "pcl_manipulation/arm5e_pc_grasp_planning_server.h"

#include <mar_perception/PCAutonomousGraspPlanning.h>

pcl_manipulation::ObjectDetectionServer::ObjectDetectionServer(
  ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  nh_(nh), nh_private_(nh_private)
{

  service_server_ = nh_private_.advertiseService("detect_object",
      &ObjectDetectionServer::serviceCallback, this);
  ROS_INFO("Service \"detect_object\" advertised.");
}

bool pcl_manipulation::ObjectDetectionServer::serviceCallback(
    DetectObjectRequest& req,
    DetectObjectResponse& res)
{
  ROS_INFO("OD called. Starting detection");
  //subscribe to pointcloud...
  sub_=nh_private_.subscribe("chatter", 1000, &ObjectDetectionServer::cloudCallback, this);
  //wait for subscriber & cloud...
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ROS_INFO("Start perception...");
  PCAutonomousGraspPlanning planner(0, 0, 0, true, cloud_);
  planner.perceive();
  //cMg should be published in TF
  //reachabillity and so on is done in client...
  return true;
}


void pcl_manipulation::ObjectDetectionServer::cloudCallback(const sensor_msgs::PointCloud2 input)
{
  //ROS Points2 message to PCL.
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(input, pcl_pc);
  //PCL Generic cloud to XYZRGB strong type.
  pcl::fromPCLPointCloud2(pcl_pc, *cloud_);
  ROS_INFO_STREAM("Acquiring cloud. Cloud size: " << cloud_->width * cloud_->height << "points.");
  sub_.shutdown();
}


int main(int argc, char **argv){
  ros::init(argc, argv, "object_detection_server");
  ros::NodeHandle nh;
  pcl_manipulation::ObjectDetectionServer ods(nh,nh);
  ros::spin();
  return 0;
}
