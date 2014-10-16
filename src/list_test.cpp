/** 
 * 3D interface to plan show a many grasps.
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

#include <mar_robot_arm5e/ARM5Arm.h>

struct GraspHypothesis{
  vpHomogeneousMatrix cMg, cMg_ik;
  double distance_score, distance_ik_score, angle_ik_score, angle_axis_score, overall_score;
};

bool sortByScore(GraspHypothesis a, GraspHypothesis b)
{
        //Using if-else statements could be better to compare. Sets priority better.
        return a.overall_score < b.overall_score;
}

double angle(vpColVector a, vpColVector b)
{
  return acos(vpColVector::dotProd(a, b) / (a.euclideanNorm() * b.euclideanNorm()));
}

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
  bool sort;

  std::string input_basename("output");
  nh.getParam("input_basename", input_basename);
  nh.param("sort", sort, false);

  ARM5Arm robot(nh, "/uwsim/joint_state", "/uwsim/joint_state_command");

  //Point Cloud load
  std::string point_cloud_file(input_basename + std::string(".pcd"));
  osgPCDLoader<pcl::PointXYZRGB> pcd_geode(point_cloud_file);
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read(point_cloud_file, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

  //Init planner
  PCAutonomousGraspPlanning planner(cloud);
  planner.perceive();
  planner.generateGraspList();

  std::cout << "Starting visualization in UWSim" << std::endl;
  //UWSim lib init
  std::string resources_data_path(".");
  nh.getParam("resources_data_path", resources_data_path);

  osgDB::Registry::instance()->getDataFilePathList().push_back(resources_data_path);
  const std::string SIMULATOR_DATA_PATH = std::string(getenv("HOME")) + "/.uwsim/data";
  osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH));
  boost::shared_ptr<osg::ArgumentParser> arguments(new osg::ArgumentParser(&argc, argv));

  //Load UWSim scene
  std::string configfile = resources_data_path + "/arm5e_gripper.xml";
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

  //Reachable position
  vpMatrix cMg_ik = planner.get_cMg().transpose();
  osg::Matrixd osg_ik_cMg(cMg.data);
  osg::MatrixTransform *ik_gt = new osg::MatrixTransform(osg_ik_cMg);
  ik_gt->addChild(UWSimGeometry::createFrame(0.003, 0.08));
  UWSimGeometry::applyStateSets(ik_gt);
  builder.getScene()->localizedWorld->addChild(ik_gt);

  //Hand frame wrt end-effector, allows for visual repositioning.
  vpHomogeneousMatrix eMh = mar_params::paramToVispHomogeneousMatrix(&nh, "eMh");
  vpHomogeneousMatrix cMe = planner.get_cMg(), ik_cMe; //*eMh.inverse();
  double count=0;

  tf::StampedTransform transform;
  bool found = false;
  while (!found)
  {
    try
    {
      listener.lookupTransform("/kinematic_base", "/stereo", ros::Time(0), transform); //Si falla espero que no modifique trasnform de ningun modo.
      found = true;
    }
    catch (const tf::TransformException & ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  vpHomogeneousMatrix bMc = VispUtils::vispHomogFromTfTransform(tf::Transform(transform));

  std::vector<GraspHypothesis> grasps;
  //Filter list...
  for (std::list<vpHomogeneousMatrix>::const_iterator ci = planner.cMg_list.begin(); ci != planner.cMg_list.end(); ++ci)
  {
    GraspHypothesis g;
    g.cMg = *ci;
    vpHomogeneousMatrix bMg = bMc * g.cMg;
    vpColVector final_joints(5), final_joints2(5);
    final_joints = robot.armIK(bMg);
    vpHomogeneousMatrix bMg_fk;
    final_joints2[0] = final_joints[0];
    final_joints2[1] = final_joints[1];
    final_joints2[2] = final_joints[2];
    final_joints2[3] = 1.57;
    final_joints2[4] = 0;
    bMg_fk = robot.directKinematics(final_joints2);
    g.cMg_ik=bMc.inverse()*bMg_fk;
    //Compute score
    g.distance_ik_score=(g.cMg.column(4) - g.cMg_ik.column(4)).euclideanNorm();
    g.angle_ik_score=abs(angle(g.cMg.column(3), g.cMg_ik.column(3)));
    g.angle_axis_score=abs(abs(angle(planner.cMo.column(2), g.cMg_ik.column(3)))-1);//Angle between cylinder axis and grasp axis.1 rad is preferred
    g.distance_score=abs((planner.cMo.column(4) - g.cMg_ik.column(4)).euclideanNorm()-0.35);//35cm is preferred
    g.overall_score=g.distance_ik_score*100+g.angle_ik_score*10+g.angle_axis_score+g.distance_score*2;//Should be argued. Know is only a matter of priority.
    grasps.push_back(g);
  }
  std::sort(grasps.begin(), grasps.end(), sortByScore);

  int index=0;
  while (ros::ok() && !view.getViewer()->done())
  {
    //UI Code
    cv::namedWindow("Grasp generator", CV_WINDOW_NORMAL);
    cv::createTrackbar("Grasp", "Grasp generator", &index, grasps.size()-1>50? 50:grasps.size()-1); //Show all  remove :? grasps.size()-1
    //UI Text Block
    std::ostringstream s1,s2,s3,s4,s5;
    s1 << "Distance score: " << grasps[index].distance_ik_score;
    s2 << "Angle ik score: " << grasps[index].angle_ik_score;
    s3 << "Ang axis score: " << grasps[index].angle_axis_score;
    s4 << "Distance to object: " << grasps[index].distance_score;
    s5 << "Overall score:" << grasps[index].overall_score;
    std::string a1(s1.str()), a2(s2.str()), a3(s3.str()), a4(s4.str()), a5(s5.str());
    cv::Mat image = cv::Mat::zeros( 300, 600, CV_8UC3 );
    cv::putText(image, a1, cv::Point(30,50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,200), 4);
    cv::putText(image, a2, cv::Point(30,85), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,200), 4);
    cv::putText(image, a3, cv::Point(30,120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,200), 4);
    cv::putText(image, a4, cv::Point(30,155), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,200), 4);
    cv::putText(image, a5, cv::Point(30,190), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,200), 4);
    imshow("Grasp generator", image);

    //Display cMg and cMg_ik
    cMe = grasps[index].cMg;
    ik_cMe = grasps[index].cMg_ik;

    osg::Matrixd osg_cMe_f(cMe.transpose().data);
    gt->setMatrix(osg_cMe_f);
    cMe = cMe * vpHomogeneousMatrix(0, 0, 0, 0, 1.57, 0).inverse();
    osg::Matrixd osg_cMe(cMe.transpose().data);
    builder.iauvFile[0]->setVehiclePosition(osg_cMe);

    osg::Matrixd osg_ik_cMe(ik_cMe.transpose().data);
    ik_gt->setMatrix(osg_ik_cMe);
    ik_cMe = ik_cMe * vpHomogeneousMatrix(0, 0, 0, 0, 1.57, 0).inverse();
    osg::Matrixd osg_ik_cMe2(ik_cMe.transpose().data);
    builder.iauvFile[1]->setVehiclePosition(osg_ik_cMe2);

    cv::waitKey(5);
    ros::spinOnce();
    view.getViewer()->frame();

  }
  return 0;
}
