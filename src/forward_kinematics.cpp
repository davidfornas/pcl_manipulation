/*
 * This program sends the DH model to the TF tree. Is not very good visually.
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#include <mar_perception/VispUtils.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpColVector.h>
#include <stdlib.h>


#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>


#include <robot_state_publisher/robot_state_publisher.h>

vpHomogeneousMatrix forwardKinematics(vpColVector q, int segment, KDL::ChainFkSolverPos_recursive fksolver) {

  vpHomogeneousMatrix wMe;

  KDL::JntArray jq(q.getRows()-1);
  for(unsigned int i=0;i<q.getRows()-1;i++)
    jq(i)=q[i];

  KDL::Frame cartpos;
  fksolver.JntToCart(jq,cartpos, segment);
  for (int i=0; i<4; i++)
    for (int j=0;j<4;j++)
      wMe[i][j]=cartpos(i,j);

  return wMe;
}

int main(int argc, char **argv) {
	
  ros::init(argc, argv, "direct_kinematics");
  ros::NodeHandle nh;


 vpColVector qx(5);
  
  if(argc!=6){
	  ROS_INFO("Not enough parameters");
	  ROS_INFO("Usage: <q0> <q1> <q2> <q3> <q4>");
	  return 0;
  }
  
  qx[0]=atof(argv[1]);
  qx[1]=atof(argv[2]);
  qx[2]=atof(argv[3]);
  qx[3]=atof(argv[4]);
  qx[4]=atof(argv[5]);

  //Definition of a kinematic chain & add segments to the chain
  KDL::Chain chain;
  //Denavit-Hartenberg model from ARM5Arm
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.08052,  M_PI_2,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.44278, 0.0 ,  0.0    , 7.98*M_PI/180     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.083 ,  M_PI_2,  0.0    , 113*M_PI/180     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0,  0.49938    , 0.0     )));

  KDL::ChainFkSolverPos_recursive fksolver (chain);
  ROS_INFO("Joints: %d, Segments: %d.", chain.getNrOfJoints(), chain.getNrOfSegments() );
  
  //End effector using forward kinematics.
  vpHomogeneousMatrix bMee = forwardKinematics(qx,-1,fksolver);
  //print bMee
  ROS_INFO_STREAM("bMee is:" << std::endl << bMee);


  /* TO VISUALIZE IT IN TF UNCOMMENT THIS

  VispToTF v;
  v.addTransform(bMee, "base", "ee", "5");
  ros::Rate rate(15);

  while(nh.ok()){

    v.publish();
    rate.sleep();
    ros::spinOnce();

  }
  return 1;*/

}
