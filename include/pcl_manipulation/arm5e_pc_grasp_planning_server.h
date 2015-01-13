#ifndef GRASP_PLANNING_OBJECT_DETECTOR_SERVER
#define GRASP_PLANNING_OBJECT_DETECTOR_SERVER


#include "pcl_manipulation/DetectObject.h"
#include "ros/ros.h"

namespace pcl_manipulation
{

class ObjectDetectorServer
{
public:

  ObjectDetectorServer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

private:

  bool serviceCallback(DetectObjectRequest& req, DetectObjectResponse& res);

  ros::NodeHandle nh_, nh_private_;
  ros::ServiceServer service_server_;
  // planner pointer * MarkerDetector detector_;
};

} // end of namespace


#endif
