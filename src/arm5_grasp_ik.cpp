/** 
 * Grasp command utility. Computes reachabillity for the ARM5e.
 *
 *  Created on: 16/07/2014
 *      Author: dfornas
 */
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <mar_perception/VispUtils.h>

//FK final position and joints publishers
tf::TransformBroadcaster *broadcaster;
ros::Publisher js_pub;

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

  broadcaster = new tf::TransformBroadcaster();
  js_pub = nh.advertise<sensor_msgs::JointState>("/planner/joint_state_command", 1);

  ros::Rate rate(1);
  tf::TransformListener listener;

  bool found = false, found_once=false;
  tf::StampedTransform transform; //Una vez tiene una trasformación la puede publicar contínuamente aunque ya no la encuentre...
  //Esto me permitiria apagar el grasp planning para ejecutar...
  tf::StampedTransform reachable_bMg,  reachable_end_bMg;
  vpColVector final_joints(5), final_joints_end(5);

  double distance=5;
  while (nh.ok())
  {

    try
    {
      listener.lookupTransform("/kinematic_base", "/cMg", ros::Time(0), transform); //Si falla espero que no modifique trasnform de ningun modo.
      found = true; found_once=true;
    }
    catch (const tf::TransformException & ex)
    {
      //As known errors often arrise, ignore
      //ROS_ERROR("%s",ex.what());
    }
    vpColVector final_joints2(5);
    vpHomogeneousMatrix bMg_fk;
    if (found)
    {
      //IK: Check reachabillity using arm only
      /*vpHomogeneousMatrix bMg = VispUtils::vispHomogFromTfTransform(tf::Transform(transform));

      vpColVector final_joints(5);
      final_joints = robot.armIK(bMg);

      final_joints2[0] = final_joints[0];
      final_joints2[1] = final_joints[1];
      final_joints2[2] = final_joints[2];
      final_joints2[3] = 1.57;
      final_joints2[4] = 0.5;/// @TODO from gripper aperture parameter
      bMg_fk = robot.directKinematics(final_joints2);
      //std::cout << "Reach to: " << bMg_fk << std::endl;
      //std::cout << "Desired: " << bMg << std::endl;
      std::cout << "Distance to desired cMg: " << (bMg.column(4) - bMg_fk.column(4)).euclideanNorm() << std::endl;
      std::cout << "Angle to desired cMg: " << angle(bMg.column(3), bMg_fk.column(3)) << std::endl;
      std::cout << "Reachable position joints: " << std::endl << final_joints2 << std::endl;

      //Publish FK of foun joint config
      tf::StampedTransform fk(VispUtils::tfTransFromVispHomog(bMg_fk), ros::Time::now(), "/kinematic_base", "/reachable_cMg");
      reachable_bMg = fk;*/

      //IK: Check reachabillity using vehicle-arm chain
      vpHomogeneousMatrix bMg = VispUtils::vispHomogFromTfTransform(tf::Transform(transform));

      vpColVector final_joints(9);
      final_joints = robot.vehicleArmIK(bMg);

      final_joints2[0] = final_joints[4];
      final_joints2[1] = final_joints[5];
      final_joints2[2] = final_joints[6];
      final_joints2[3] = 1.57;
      final_joints2[4] = 0.5;/// @TODO from gripper aperture parameter
      bMg_fk = robot.directKinematics(final_joints2);
      //std::cout << "Reach to: " << bMg_fk << std::endl;
      //std::cout << "Desired: " << bMg << std::endl;
      std::cout << "Distance to desired cMg: " << (bMg.column(4) - bMg_fk.column(4)).euclideanNorm() << std::endl;
      std::cout << "Angle to desired cMg: " << angle(bMg.column(3), bMg_fk.column(3)) << std::endl;
      std::cout << "Reachable position joints: " << std::endl << final_joints2 << std::endl;

      //Publish FK of foun joint config
      tf::StampedTransform fk(VispUtils::tfTransFromVispHomog(bMg_fk), ros::Time::now(), "/kinematic_base", "/reachable_cMg");
      reachable_bMg = fk;
    }
    //Always send last valid transform and joint_state
    reachable_bMg.stamp_=ros::Time::now();
    if(found_once) broadcaster->sendTransform(reachable_bMg);


    //DF: part 2 end
    vpColVector final_joints2_end(5);
    if (found)
    {
      //IK: Check reachabillity...
      vpHomogeneousMatrix bMg_end = bMg_fk * vpHomogeneousMatrix(0, 0, distance/100.0, 0, 0, 0);

      vpColVector final_joint_ends(5);
      final_joints_end = robot.armIK(bMg_end);
      vpHomogeneousMatrix bMg_fk_end;

      final_joints2_end[0] = final_joints_end[0];
      final_joints2_end[1] = final_joints_end[1];
      final_joints2_end[2] = final_joints_end[2];
      final_joints2_end[3] = 1.57;
      final_joints2_end[4] = 0;
      bMg_fk_end = robot.directKinematics(final_joints2_end);
      //Publish FK of foun joint config
      tf::StampedTransform fk_end(VispUtils::tfTransFromVispHomog(bMg_fk_end), ros::Time::now(), "/kinematic_base", "/reachable_cMg_end");
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

    rate.sleep();
    ros::spinOnce();
    nh.getParam("distance", distance);
  }
  return 0;
}
