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

vpHomogeneousMatrix directKinematics(vpColVector q, int segment, KDL::ChainFkSolverPos_recursive fksolver) {

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
	
  ros::init(argc, argv, "test_kdl_dh");
  ros::NodeHandle nh;
  
  VispToTF v;

  double qx[5];
   
  qx[0]=atof(argv[1]);
  qx[1]=atof(argv[2]);
  qx[2]=atof(argv[3]);
  qx[3]=atof(argv[4]);
  qx[4]=atof(argv[5]);
  
  vpColVector q(5); q.data=qx;

  //Definition of a kinematic chain & add segments to the chain
  KDL::Chain chain;
  //Denavit-Hartenberg model from ARM5Arm
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.08052,  M_PI_2,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.44278, 0.0 ,  0.0    , 7.98*M_PI/180     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.083 ,  M_PI_2,  0.0    , 113*M_PI/180     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0,  0.49938    , 0.0     )));

  KDL::ChainFkSolverPos_recursive fksolver (chain);
  ROS_INFO("Joints: %d, Segments: %d.", chain.getNrOfJoints(), chain.getNrOfSegments() );
  
  double q1=atof(argv[1]), q2=atof(argv[2]), q3=atof(argv[3]), q4=atof(argv[4]);
  //Manually get forward kinematics.
  vpHomogeneousMatrix bMf1= vpHomogeneousMatrix(0,0,0,0,0,q1) * vpHomogeneousMatrix(0.08052,0,0,0,0,0) * vpHomogeneousMatrix(0,0,0,M_PI_2,0,0);
  vpHomogeneousMatrix bMf2 = vpHomogeneousMatrix(0,0,0,0,0,7.98*M_PI/180 + q2) * vpHomogeneousMatrix(0.4427,0,0,0,0,0);
  vpHomogeneousMatrix bMf3 = vpHomogeneousMatrix(0,0,0,0,0,113*M_PI/180 + q3) * vpHomogeneousMatrix(-0.083,0,0,0,0,0) * vpHomogeneousMatrix(0,0,0,M_PI_2,0,0); 
  vpHomogeneousMatrix bMf4 = vpHomogeneousMatrix(0,0,0,0,0,q4) * vpHomogeneousMatrix(0,0,0.49938,0,0,0) ;
    
  //Moving the worl to see better.
  vpHomogeneousMatrix wMb(0.50,0,0,M_PI,0,0);
    
  //.addTransform(wMb, "world", "base", "w");
  bool eefOnly=false;
  
  if(!eefOnly){  
  
	  v.addTransform(vpHomogeneousMatrix(0,0,0,0,0,0), "base", "Slew", "d");
	  v.addTransform(bMf1, "Slew", "Shoulder", "1");
	  v.addTransform(bMf2, "Shoulder", "Elbow", "2");
	  v.addTransform(bMf3, "Elbow", "Wrist", "3");
	  v.addTransform(bMf4, "Wrist", "EE", "4");

  }
  
  //Solving FK for each joints is equivalent.
  /*vpHomogeneousMatrix bMf1 = directKinematics(q,1,fksolver);
  vpHomogeneousMatrix bMf2 = directKinematics(q,2,fksolver);
  vpHomogeneousMatrix bMf3 = directKinematics(q,3,fksolver);
  vpHomogeneousMatrix bMf4 = directKinematics(q,4,fksolver); //End effector, seria lo mismo poner -1
  v.addTransform(bMf1, "base", "Slew", "1");
  v.addTransform(bMf2, "base", "Shoulder", "2");
  v.addTransform(bMf3, "base", "Elbow", "3");
  v.addTransform(bMf4, "base", "End_effector", "4");*/
  
  //End effector using forward kinematics.
  vpHomogeneousMatrix bMee = directKinematics(q,-1,fksolver);
  v.addTransform(bMee, "base", "ee", "5");
  std::cout << bMee << std::endl;

  ros::Rate rate(15);

  while(nh.ok()){

    v.publish();
    rate.sleep();
    ros::spinOnce();

  }
  return 1;

}
