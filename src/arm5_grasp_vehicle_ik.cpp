/** 
 * Grasp command utility. Computes reachabillity for the ARM5e.
 *
 *  Created on: 16/07/2014
 *      Author: dfornas
 */
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <mar_perception/VispUtils.h>

#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Matrix>


double angle(vpColVector a, vpColVector b)
{
  return acos(vpColVector::dotProd(a, b) / (a.euclideanNorm() * b.euclideanNorm()));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm5_grasp_exec");
  ros::NodeHandle nh;
  //Create ARM5Robot
  ARM5Arm robot(nh, "/uwsim/joint_state", "/uwsim/joint_state_command");

  tf::TransformBroadcaster *broadcaster = new tf::TransformBroadcaster();
  ros::Publisher js_pub = nh.advertise<sensor_msgs::JointState>("/uwsim/joint_state_command", 1);
  ros::Publisher position_pub=nh.advertise<nav_msgs::Odometry>("/dataNavigator",1);

  ros::Rate rate(1);
  tf::TransformListener listener;


  bool found = false, found_once=false;
  tf::StampedTransform transform; //Una vez tiene una trasformación la puede publicar contínuamente aunque ya no la encuentre...
  //Esto me permitiria apagar el grasp planning para ejecutar...
  tf::StampedTransform reachable_bMg,  reachable_end_bMg, vtrans;
  vpColVector final_joints(5), final_joints_end(5);

  double distance=5;
  while (nh.ok())
  {

    try
    {
      listener.lookupTransform("/vehicle", "/cMg", ros::Time(0), transform); //Si falla espero que no modifique transform de ningun modo.
      //kinematic_base <=> vehicle
      found = true; found_once=true;
    }
    catch (const tf::TransformException & ex)
    {
      //As known errors often arrise, ignore
      //ROS_ERROR("%s",ex.what());
    }
    vpColVector final_joints(8),final_joints2(5);
    vpHomogeneousMatrix bMg_fk;
    if (found)
    {
      //IK: Check reachabillity using vehicle-arm chain
      vpHomogeneousMatrix bMg = VispUtils::vispHomogFromTfTransform(tf::Transform(transform));

      //REMEMBER: Enable full Jacobian in JntToCart
      final_joints = robot.vehicleArmIK(bMg);
      std::cout << "Reachable position vehicle config: " << std::endl << final_joints << std::endl;

      final_joints2[0] = final_joints[4];
      final_joints2[1] = final_joints[5];
      final_joints2[2] = final_joints[6];
      final_joints2[3] = 1.57;
      final_joints2[4] = 0.5;/// @TODO from gripper aperture parameter
      bMg_fk = robot.directKinematics(final_joints2);
      //Publish FK of found joint config
      tf::StampedTransform fk(VispUtils::tfTransFromVispHomog(bMg_fk), ros::Time::now(), "/kinematic_base2", "/reachable_cMg");//kinematic_base <=> vehicle
      reachable_bMg = fk;

      vpHomogeneousMatrix x(final_joints[0],0,0,0,0,0);
      vpHomogeneousMatrix y(0,final_joints[1],0,0,0,0);
      vpHomogeneousMatrix z(0,0,final_joints[2],0,0,0);
      vpHomogeneousMatrix yaw(0,0,0,0,0,final_joints[3]);
      vpHomogeneousMatrix t=x*y*z*yaw;
      tf::StampedTransform v(VispUtils::tfTransFromVispHomog(t), ros::Time::now(), "/vehicle", "/vehicle2");//kinematic_base <=> vehicle
      vtrans=v;



    }
    //Always send last valid transform and joint_state
    reachable_bMg.stamp_=ros::Time::now();
    vtrans.stamp_=ros::Time::now();

    if(found_once) broadcaster->sendTransform(reachable_bMg);
    if(found_once) broadcaster->sendTransform(vtrans);


    //DF: part 2 end
    vpColVector final_joints2_end(5);
    if (found)
    {
      //IK: Check reachabillity...
      vpHomogeneousMatrix bMg_end = bMg_fk * vpHomogeneousMatrix(0, 0, distance/100.0, 0, 0, 0);

      vpColVector final_joint_ends(5);
      final_joints_end = robot.vehicleArmIK(bMg_end);
      vpHomogeneousMatrix bMg_fk_end;

      final_joints2_end[0] = final_joints_end[4];
      final_joints2_end[1] = final_joints_end[5];
      final_joints2_end[2] = final_joints_end[6];
      final_joints2_end[3] = 1.57;
      final_joints2_end[4] = 0;
      bMg_fk_end = robot.directKinematics(final_joints2_end);
      //Publish FK of foun joint config
      tf::StampedTransform fk_end(VispUtils::tfTransFromVispHomog(bMg_fk_end), ros::Time::now(), "/kinematic_base2", "/reachable_cMg_end");
      reachable_end_bMg = fk_end;
    }
    //Always send last valid transform and joint_state
    reachable_end_bMg.stamp_=ros::Time::now();
    if(found_once) broadcaster->sendTransform(reachable_end_bMg);

    sensor_msgs::JointState js;
    js.name.push_back(std::string("Slew"));
    js.position.push_back(final_joints2[0]);
    js.name.push_back(std::string("Shoulder"));
    js.position.push_back(final_joints2[1]);
    js.name.push_back(std::string("Elbow"));
    js.position.push_back(final_joints2[2]);
    js.name.push_back(std::string("JawRotate"));
    js.position.push_back(final_joints2[3]);
    js.name.push_back(std::string("JawOpening"));
    js.position.push_back(final_joints2[4]);
    js_pub.publish(js);

    osg::Matrixd Rz;
    Rz.makeRotate(final_joints[3],0,0,1);
    osg::Quat rot=Rz.getRotate();

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x=0.3+final_joints[0];
    odom.pose.pose.position.y=0.5+final_joints[1];
    odom.pose.pose.position.z=2.8+final_joints[2];
    odom.pose.pose.orientation.x=rot.x();//YAW FROM final_joints2[2]
    odom.pose.pose.orientation.y=rot.y();
    odom.pose.pose.orientation.z=rot.z();
    odom.pose.pose.orientation.w=rot.w();
    position_pub.publish(odom);

    rate.sleep();
    ros::spinOnce();
    nh.getParam("distance", distance);
  }
  return 0;
}
