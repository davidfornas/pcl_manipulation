/** 
 * Grasp execution utility. Moves the ARM5e to the goal cMg.
 *
 *  Created on: 16/07/2014
 *      Author: dfornas
 */
#include <tf/transform_listener.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <mar_perception/VispUtils.h>

ARM5Arm* robot_;

///Moved from GRASPER
///Open or close the gripper accounting for current limits
void openGripper(double velocity, double aperture, double current)
{
  vpColVector vel(5), current_joints;
  vel = 0;
  vel[4] = velocity;
  robot_->getJointValues(current_joints);
  if (velocity > 0)
  {
    while (current_joints[4] < aperture && ros::ok() && robot_->getCurrent() < current)
    {
      robot_->setJointVelocity(vel);
      ros::spinOnce();
      robot_->getJointValues(current_joints);
    }
  }
  else
  {
    while (current_joints[4] > aperture && ros::ok() && robot_->getCurrent() < current)
    {
      robot_->setJointVelocity(vel);
      ros::spinOnce();
      robot_->getJointValues(current_joints);
    }
  }
}

///Moved from GRASPER
///Reach a position with respect to a given 
void reachPosition(vpHomogeneousMatrix cMgoal, vpHomogeneousMatrix bMc)
{
  vpHomogeneousMatrix bMe, cMe;
  //joint_offset_->get_bMc(bMc);

  robot_->getPosition(bMe);
  cMe = bMc.inverse() * bMe;
  while ((cMe.column(4) - cMgoal.column(4)).euclideanNorm() > 0.02 && ros::ok())
  {
    //std::cout << "Error: " << (cMe.column(4) - cMgoal.column(4)).euclideanNorm() << std::endl;
    vpColVector xdot(6);
    xdot = 0;
    vpHomogeneousMatrix eMgoal = cMe.inverse() * cMgoal;
    xdot[0] = eMgoal[0][3] * 0.5;
    xdot[1] = eMgoal[1][3] * 0.5;
    xdot[2] = eMgoal[2][3] * 0.5;
    robot_->setCartesianVelocity(xdot);
    ros::spinOnce();

    robot_->getPosition(bMe);
    cMe = bMc.inverse() * bMe;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm5_grasp_exec");
  ros::NodeHandle nh;

  std::string joint_state, joint_state_command;
  double max_current, velocity_aperture, velocity_closure, gripper_opened, gripper_closed;
  bool pick;

  nh.getParam("joint_state", joint_state);
  nh.getParam("joint_state_command", joint_state_command);
  nh.getParam("max_current", max_current);
  nh.getParam("velocity_aperture", velocity_aperture);
  nh.getParam("velocity_closure", velocity_closure);
  nh.getParam("gripper_opened", gripper_opened);
  nh.getParam("gripper_closed", gripper_closed);
  nh.getParam("pick", pick);

  //Create ARM5Robot  
  robot_ = new ARM5Arm(nh, joint_state, joint_state_command);

  ros::Rate rate(20);
  tf::TransformListener listener;
  tf::StampedTransform transform, transform2;
  //Get bMc first
  bool bMc_found = false;
  while (nh.ok())
  {
    while (nh.ok() && !bMc_found)
    {
      try
      {
        listener.lookupTransform("/kinematic_base", "/stereo", ros::Time(0), transform);
        bMc_found = true;
      }
      catch (const tf::TransformException & ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      rate.sleep();
      ros::spinOnce();
    }
    vpHomogeneousMatrix bMc = VispUtils::vispHomogFromTfTransform(tf::Transform(transform));
    // Then get cMg goal pose
    bool cMgoal_found = false;
    while (nh.ok() && !cMgoal_found)
    {
      try
      {
        listener.lookupTransform("/stereo", "/reachable_cMg", ros::Time(0), transform2);
        cMgoal_found = true;
      }
      catch (const tf::TransformException & ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      rate.sleep();
      ros::spinOnce();
    }
    vpHomogeneousMatrix cMgoal = VispUtils::vispHomogFromTfTransform(tf::Transform(transform2));
    //Mover a posición a través de movimiento lineal cartesiano.
    reachPosition(cMgoal, bMc);
    ROS_INFO("Starting position reached");

    if (pick)
    {
      ROS_INFO("Opening gripper");

      openGripper(velocity_aperture, gripper_opened, max_current);

      ROS_INFO("Moving forward 12 cms (Z Axis)");
      //reachCloseposition... Currently there is any reachabillity check so the distance should be
      //near enough and the previous goal away from the workspace limits.
      vpHomogeneousMatrix new_cMgoal = cMgoal * vpHomogeneousMatrix(0, 0, 0.12, 0, 0, 0);
      reachPosition(new_cMgoal, bMc);

      ROS_INFO("Close position reached, Closing the gripper (until a current is sensed)");
      openGripper(velocity_closure, gripper_closed, max_current);

      ROS_INFO("Finished. Going home now...");
      //cMgoal = cMgoal * vpHomogeneousMatrix(0, 0, -0.4, 0, 0, 0);
      reachPosition(cMgoal, bMc);

      ROS_INFO("Grasp ended");
      pick=false;//This avoids dropping the amphora in real scenarios.One time only program.
    }

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
