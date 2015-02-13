/** 
 * 3D interface to plan a grasp using a point cloud.
 *
 *  Created on: 23/01/2013
 *      Author: dfornas
 */
#include <uwsim/ConfigXMLParser.h>
#include <string>

//MAR
#include <mar_perception/PCAutonomousGraspPlanning.h>
#include <mar_perception/VispUtils.h>
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
#include <tf/transform_listener.h>
VispToTF * vispTf;

/** Plans a grasp on a point cloud and visualizes it using UWSim
 * Expects the following params to be set in the ROS parameter server:
 * input_basename: The full path to the input files without PCD extension, ie. /tmp/scan_2
 * resources_data_path: The path the folder with the models for UWSim
 * eMh: a six-element vector representing the hand frame wrt the end-effector frame. [x y z roll pitch yaw] format; now UNUSED
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "arm5e_pc_grasp_planning");
  ros::NodeHandle nh;
  //Listen for rechable frame  
  tf::TransformListener listener;

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
  osgPCDLoader<pcl::PointXYZRGB> pcd_geode(point_cloud_file);
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read(point_cloud_file, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

  //Init planner
  PCAutonomousGraspPlanning planner(angle, rad, along, alignedGrasp, cloud);
  planner.perceive();

  std::cout << "Starting visualization in UWSim" << std::endl;
  //UWSim lib init
  std::string resources_data_path(".");
  nh.getParam("resources_data_path", resources_data_path);

  osgDB::Registry::instance()->getDataFilePathList().push_back(resources_data_path);
  const std::string SIMULATOR_DATA_PATH = std::string(getenv("HOME")) + "/.uwsim/data";
  osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH));
  boost::shared_ptr<osg::ArgumentParser> arguments(new osg::ArgumentParser(&argc, argv));

  //Load UWSim scene
  std::string configfile = resources_data_path + "/arm5e_arm.xml";
  ConfigFile config(configfile);
  SceneBuilder builder(arguments);
  builder.loadScene(config);
  //Load view
  ViewBuilder view(config, &builder, arguments);
  osg::ref_ptr<osgGA::TrackballManipulator> tb = new osgGA::TrackballManipulator;
  tb->setHomePosition(osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, -1), osg::Vec3f(-1, 0, 0));
  view.getViewer()->setCameraManipulator(tb);
  view.init();
  view.getViewer()->realize();
  view.getViewer()->frame();

  osgViewer::Viewer::Windows windows;
  view.getViewer()->getWindows(windows);
  windows[0]->setWindowName("UWSim Grasp Specification");

  UWSimGeometry::applyStateSets(pcd_geode.getGeode());
  builder.getScene()->localizedWorld->addChild(pcd_geode.getGeode());

  //Scene dynamic objects
  //Desired position gripper...
  vpMatrix cMg = planner.get_cMg().transpose();
  osg::Matrixd osg_cMg(cMg.data);
  osg::MatrixTransform *gt = new osg::MatrixTransform(osg_cMg);
  gt->addChild(UWSimGeometry::createFrame(0.005, 0.1));
  UWSimGeometry::applyStateSets(gt);
  builder.getScene()->localizedWorld->addChild(gt);
  vpHomogeneousMatrix cMe = planner.get_cMg(); //*eMh.inverse();

  vpHomogeneousMatrix ik_cMe;
  bool found = false;
  tf::StampedTransform transform;
  //Cheack reachabillity
  try
  {
    listener.lookupTransform("/stereo", "/kinematic_base", ros::Time(0), transform); //Si falla espero que no modifique trasnform de ningun modo.
    found = true;
  }
  catch (const tf::TransformException & ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  if (found)
  {
    //IK: Check reachabillity...
    ik_cMe = VispUtils::vispHomogFromTfTransform(tf::Transform(transform));
  }
  osg::Matrixd osg_ik_cMe(ik_cMe.transpose().data);
  //ik_gt->setMatrix(osg_ik_cMe);
  //TWO HANDS or only frames? builder.iauvFile[0]->setVehiclePosition(osg_cMe);
  //ik_cMe = ik_cMe * vpHomogeneousMatrix(0, 0, 0, 0, 1.57, 0).inverse();
  osg::Matrixd osg_ik_cMe2(ik_cMe.transpose().data);
  builder.iauvFile[1]->setVehiclePosition(osg_ik_cMe2);

  // Show muliple possibilities
  /// @todo 
  bool multiples = true;

  //Distance to reach
  int distance=8;

  while (ros::ok() && !view.getViewer()->done())
  {
    //Interface
    cv::namedWindow("Grasp configuration", CV_WINDOW_NORMAL);
    cv::createTrackbar("Radius", "Grasp configuration", &(planner.irad), 100);
    cv::createTrackbar("Angle", "Grasp configuration", &(planner.iangle), 360);
    cv::createTrackbar("Distance", "Grasp configuration", &(planner.ialong), 100);
    cv::createTrackbar("Aligned grasp?", "Grasp configuration", &(planner.ialigned_grasp), 1);
    cv::createTrackbar("A. Distance", "Grasp configuration", &distance, 20);
    //Compute adn display new grasp frame
    planner.recalculate_cMg();

    //Desired cfg
    cMe = planner.get_cMg(); //*eMh.inverse();
    osg::Matrixd osg_cMe_f(cMe.transpose().data);
    gt->setMatrix(osg_cMe_f);

    cMe = cMe * vpHomogeneousMatrix(0, 0, 0, 0, 1.57, 0).inverse();
    osg::Matrixd osg_cMe(cMe.transpose().data);
    builder.iauvFile[0]->setVehiclePosition(osg_cMe);

    cv::waitKey(5);
    nh.setParam("distance", distance);
    ros::spinOnce();
    view.getViewer()->frame();
  }
  return 0;
}
