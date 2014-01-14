/*
 * Plans a 3D grasp on the point cloud autonomously.
 *
 *  Created on: 23/01/2013
 *      Author: dfornas
 */
#include <uwsim/ConfigXMLParser.h>

//MAR
#include <mar_perception/PCAutonomousGraspPlanning.h>
#include <mar_ros_bridge/mar_params.h>

//UWSim + OSG
#include <uwsim/SceneBuilder.h>
#include <uwsim/ViewBuilder.h>
#include <uwsim/UWSimUtils.h>
#include <uwsim/osgPCDLoader.h>
#include <osgGA/TrackballManipulator>


//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//Quick GUI form OpoenCV
#include <vector>
#include <opencv2/highgui/highgui.hpp>

//TF
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>


//TF Broadcaster to visualize target pose, could be a marker also.
tf::TransformBroadcaster *broadcaster;

double i=0;
void publish_cMg(vpHomogeneousMatrix cMe){
   
  vpHomogeneousMatrix test(0,0,0,M_PI/2,0,0); //vpHomogeneousMatrix test(0,0,0,0,M_PI/2,0);
  vpHomogeneousMatrix test2(0,0,0,0,0,M_PI/2);
  cMe=cMe*test*test2;	

  std::cerr << "cMg pre:" << std::endl << cMe;

  //cMe to PoseStamped
  vpQuaternionVector q;
  vpTranslationVector V;
  cMe.extract(q);
  cMe.extract(V);

  tf::Vector3 p(V[0], V[1], V[2]);
  tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  tf::Transform pose;
  pose.setOrigin(p);
  pose.setRotation(tf_q);

  tf::StampedTransform t(pose, ros::Time::now(), "/camera", "/grasp_frame");
  broadcaster->sendTransform(t);

}


/** Plans a grasp on a point cloud and visualizes it using UWSim
 * Expects the following params to be set in the ROS parameter server:
 * input_basename: The full path to the input files without PCD extension, ie. /tmp/scan_2
 * resources_data_path: The path the folder with the models for UWSim
 * eMh: a six-element vector representing the hand frame wrt the end-effector frame. [x y z roll pitch yaw] format
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "arm5e_pc_grasp_planning");
  ros::NodeHandle nh;

  broadcaster = new tf::TransformBroadcaster();

  //Angulo de agarre
  double angle, rad, along;
  bool alignedGrasp;

  std::string input_basename("output");
  nh.getParam("input_basename", input_basename);
  
  nh.param("alignedGrasp", alignedGrasp, false);
  nh.getParam("angle", angle);
  nh.getParam("rad", rad);
  nh.getParam("along", along);

  //Point Cloud load
  std::string point_cloud_file(input_basename+std::string(".pcd"));
  osgPCDLoader<pcl::PointXYZRGB> pcd_geode(point_cloud_file);

  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read (point_cloud_file, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
  
  //Auto planner
  PCAutonomousGraspPlanning planner(angle, rad, along, alignedGrasp, cloud);
  planner.perceive();

  std::cout << "Planned grasp frame with respect to camera is: " << std::endl << planner.get_cMg() << std::endl;
  std::cout << "Starting visualization in UWSim" << std::endl;

  //Viz resources
  std::string resources_data_path(".");
  nh.getParam("resources_data_path", resources_data_path);
  osgDB::Registry::instance()->getDataFilePathList().push_back(resources_data_path);

  boost::shared_ptr<osg::ArgumentParser> arguments(new osg::ArgumentParser(&argc,argv));
  std::string configfile=resources_data_path+"/arm5e_gripper.xml";
  ConfigFile config(configfile);

  SceneBuilder builder(arguments);
  builder.loadScene(config);

  ViewBuilder view(config, &builder, arguments);

  osg::ref_ptr<osgGA::TrackballManipulator> tb = new osgGA::TrackballManipulator;
  tb->setHomePosition( osg::Vec3f(0,0,0), osg::Vec3f(0,0,-1), osg::Vec3f(-1,0,0) );
  view.getViewer()->setCameraManipulator( tb );
  view.init();
  view.getViewer()->realize();
  view.getViewer()->frame();

  osgViewer::Viewer::Windows windows;
  view.getViewer()->getWindows(windows);
  windows[0]->setWindowName("UWSim");

  UWSimGeometry::applyStateSets(pcd_geode.getGeode());
  builder.getScene()->localizedWorld->addChild(pcd_geode.getGeode());

  vpMatrix cMg=planner.get_cMg().transpose();
  osg::Matrixd osg_cMg(cMg.data);
  osg::MatrixTransform *gt=new osg::MatrixTransform(osg_cMg);
  gt->addChild(UWSimGeometry::createFrame(0.005, 0.1));
  UWSimGeometry::applyStateSets(gt);
  builder.getScene()->localizedWorld->addChild(gt);

  
  //Hand frame wrt end-effector
  vpHomogeneousMatrix eMh=mar_params::paramToVispHomogeneousMatrix(&nh, "eMh");
  vpHomogeneousMatrix cMe=planner.get_cMg();///*eMh.inverse();

  publish_cMg(cMe);

  while( ros::ok() && !view.getViewer()->done())
  {
	cv::namedWindow("Grasp configuration", CV_WINDOW_NORMAL );
  	cv::createTrackbar( "Radius", "Grasp configuration", &(planner.irad), 100 );
  	cv::createTrackbar( "Angle", "Grasp configuration", &(planner.iangle), 360 );
        cv::createTrackbar( "Distance", "Grasp configuration", &(planner.ialong), 100 );
        cv::createTrackbar( "Aligned grasp?", "Grasp configuration", &(planner.ialigned_grasp), 1 );

        planner.recalculate_cMg();
        cMe=planner.get_cMg();///*eMh.inverse();
        osg::Matrixd osg_cMe(cMe.transpose().data);
        gt->setMatrix(osg_cMe);
        publish_cMg(cMe);
        builder.iauvFile[0]->setVehiclePosition(osg_cMe);
        cv::waitKey(5);
    
        ros::spinOnce();
        view.getViewer()->frame();
  }

  return 1;
}
