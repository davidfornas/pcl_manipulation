//Copy cloud to disk
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/make_shared.hpp>
#include <visp/vpImageIo.h>


// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <mar_perception/VirtualImage.h>

ros::Publisher pub;
VirtualImagePtr vPtr;
vpImage<vpRGBa>  Ic;
bool once=true;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  if(!once)return;
  once=false;

  vPtr->acquire(Ic);
  sensor_msgs::PointCloud2 output;

  // Perform the actual filtering
  //pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  //sor.setInputCloud (input);
  //sor.setLeafSize (0.01, 0.01, 0.01);
  //sor.filter (output);

  //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  //viewer.showCloud (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(cloud));

  //while (!viewer.wasStopped ())
  //  {
  // }
  //
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //pcl_conversions::toPCL(*input, cloud);
  //pcl::SomePCLFunction(pcl_pc);

  //pcl::PointCloud<pcl::PointXYZRGB> cloud(*input);
  pcl::fromROSMsg (*input, cloud);

  //boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > cloud_filtered_x =  //boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud);
  //Outlier removal
  //pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  // build the filter
  //outrem.setInputCloud(cloud_filtered_x);
  //outrem.setRadiusSearch(0.08);
  //outrem.setMinNeighborsInRadius (10);

  // apply filter
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy_out (new pcl::PointCloud<pcl::PointXYZRGB>);
  //ROS_INFO("Applying outlier removal filter...");
  //outrem.filter (*cloud_filtered_xy_out);
  //ROS_ERROR("Done");

  //Downsampling the pointcloud
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  //sor.setInputCloud (cloud_filtered_xy_out);
  //sor.setLeafSize (0.04, 0.04, 0.04);
  //sor.filter (*cloud_downsampled);

  // pcl::ModelCoefficients coefficients;
  // pcl::PointIndices inliers;
  // // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (0.01);

  // seg.setInputCloud (cloud.makeShared ());
  // seg.segment (inliers, coefficients);

  //INVERSE OR NOT
  for(int i=0;i<cloud.size();i++)
    cloud.points[i].z=/**-**/cloud.points[i].z;

  std::string filename("output.pcd");
  pcl::io::savePCDFile(filename, cloud, false);
  vpImageIo::write(Ic, "output.ppm");
  //Save camera parameters
  vpMatrix::saveMatrix("output.cam", vPtr->K.get_K());

  ROS_INFO("The cloud is saved((not inversed)).");
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  vPtr = VirtualImagePtr(new VirtualImage(nh,"stereo_down/left/image_rect", "stereo_down/left/camera_info", 1));
  vPtr->open(Ic) ;
  vPtr->acquire(Ic);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);
  ros::spin ();
}


