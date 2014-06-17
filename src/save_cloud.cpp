/*
 * Copy a pointCloud2 msg to disk. May do some filtering.
 *
 *  Created on: 20/01/2013
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/make_shared.hpp>
#include <visp/vpImageIo.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//Filters Downsampling
#include <pcl/filters/voxel_grid.h>
//sor outliers
#include <pcl/filters/statistical_outlier_removal.h>

#include <mar_perception/VirtualImage.h>

ros::Publisher pub;
VirtualImagePtr vPtr;
vpImage<vpRGBa>  Ic;
bool once=true;
bool filtering=false;

void cloud_cb (const sensor_msgs::PointCloud2 input)
{
  if(!once)return;
  once=false;

  vPtr->acquire(Ic);
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2), cloud_f1(new pcl::PCLPointCloud2), cloud_f2(new pcl::PCLPointCloud2);
  
  pcl_conversions::toPCL(input, *cloud);

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

    // Perform the actual filtering
    pcl::VoxelGrid< pcl::PCLPointCloud2 > vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01, 0.01, 0.01);
    vg.filter (*cloud_f1);
    
  //Outlier removal
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_f1);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_f2);


//passthrough also possible
//http://pointclouds.org/documentation/tutorials/reading_pcd.php#reading-pcd
  //INVERSE OR NOT
  //for(int i=0;i<cloud->width;i++)
  //  cloud->data[i].z=-cloud->data[i].z;

  std::cerr << "PointCloud before filtering: " << cloud_f1->width * cloud_f1->height 
       << " data points (" << pcl::getFieldsList (*cloud_f1) << ").";
//http://pointclouds.org/documentation/tutorials/reading_pcd.php#reading-pcd

  std::string filename("output.pcd");
  pcl::io::savePCDFile(filename, *cloud_f1);
  vpImageIo::write(Ic, "output.ppm");
  //Save camera parameters
  vpMatrix::saveMatrix("output.cam", vPtr->K.get_K());

  ROS_INFO("The cloud is saved((not inversed)).");
}

int main (int argc, char** argv) {
  
  ros::init (argc, argv, "point_cloud_saver");
  ros::NodeHandle nh;

  vPtr = VirtualImagePtr(new VirtualImage(nh,"camera/left/image_rect", "camera/left/camera_info", 1));
  vPtr->open(Ic) ;
  vPtr->acquire(Ic);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);
  ros::spin ();
}


