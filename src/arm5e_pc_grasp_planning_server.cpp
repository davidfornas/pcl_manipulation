#include "pcl_manipulation/arm5e_pc_grasp_planning_server.h"

pcl_manipulation::ObjectDetectorServer::ObjectDetectorServer(
  ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  nh_(nh), nh_private_(nh_private)
{
  //detector_.loadSettings(nh_private_);

  service_server_ = nh_private_.advertiseService("detect_marker",
      &ObjectDetectorServer::serviceCallback, this);
  ROS_INFO("Service \"detect_marker\" advertised.");
}

bool pcl_manipulation::ObjectDetectorServer::serviceCallback(
    DetectObjectRequest& req,
    DetectObjectResponse& res)
{
  //detector_.setCameraInfo(req.camera_info, req.rectified);
  //detector_.detect(req.image, res.markers);
  return true;
}



int main(void){



  return 0;
}
