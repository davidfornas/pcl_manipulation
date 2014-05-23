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

using namespace KDL;

vpHomogeneousMatrix /*ARM5Arm::*/directKinematics(vpColVector q, int segment, KDL::ChainFkSolverPos_recursive fksolver) {

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
  ros::init(argc, argv, "test_tf");
  ros::NodeHandle nh;

  VispToTF v;

  ros::Rate rate(15);

  double qx[5];
/*
  qx[0]=atof(argv[1]);
  qx[1]=atof(argv[2]);
  qx[2]=atof(argv[3]);
  qx[3]=atof(argv[4]);
  qx[4]=atof(argv[5]);
  */
  vpColVector q(5); q.data=qx;

  //Definition of a kinematic chain & add segments to the chain
  KDL::Chain chain;

  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.08052,  M_PI_2,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.44278, 0.0 ,  0.0    , 7.98*M_PI/180     )));//7.98
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.083 ,  M_PI_2,  0.0    , 113*M_PI/180     )));//127.02
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0,  0.49938    , 0.0     )));

  KDL::ChainFkSolverPos_recursive fksolver (chain);

  ROS_INFO("Joints: %d, Segments: %d., %f", chain.getNrOfJoints(), chain.getNrOfSegments(), atof(argv[2]));
  
  KDL::Frame f=chain.getSegment(1).pose(0);
  double r,p,y;
  
  f.M.GetRPY(r,p,y);
  vpHomogeneousMatrix bMf1(f.p.x(), f.p.y(), f.p.z(),r,p,y);
  
  f=chain.getSegment(2).pose(0);
  f.M.GetRPY(r,p,y);
  vpHomogeneousMatrix bMf2(f.p.x(), f.p.y(), f.p.z(),r,p,y);
 
  f=chain.getSegment(3).pose(0);
  f.M.GetRPY(r,p,y);
  vpHomogeneousMatrix bMf3(f.p.x(), f.p.y(), f.p.z(),r,p,y);
  
  f=chain.getSegment(4).pose(0);
  f.M.GetRPY(r,p,y);
  vpHomogeneousMatrix bMf4(f.p.x(), f.p.y(), f.p.z(),r,p,y);
 
  
  /*vpHomogeneousMatrix bMf1= vpHomogeneousMatrix(0,0,0.08052,0,0,0) * vpHomogeneousMatrix(0,0,0,0,0,M_PI_2);
  vpHomogeneousMatrix bMf2 = vpHomogeneousMatrix(0,0,0.4427,0,0,0) * vpHomogeneousMatrix(0,0,0,7.98*M_PI/180,0,0);
  vpHomogeneousMatrix bMf3 = vpHomogeneousMatrix(0,0,-0.083,0,0,0) * vpHomogeneousMatrix(0,0,0,0,0,M_PI_2) * vpHomogeneousMatrix(0,0,0,113*M_PI/180,0,0); //End effector, seria lo mismo poner -1
  vpHomogeneousMatrix bMf4 = vpHomogeneousMatrix(0,0,0,0.49938,0,0);*/

  v.addTransform(bMf1, "base", "Slew", "1");
  v.addTransform(bMf2, "Slew", "Shoulder", "2");
  v.addTransform(bMf3, "Shoulder", "Elbow", "3");
  v.addTransform(bMf4, "Elbow", "End_effector", "4");


  //Segments start in 0?
  /*vpHomogeneousMatrix bMf1 = directKinematics(q,1,fksolver);
  vpHomogeneousMatrix bMf2 = directKinematics(q,2,fksolver);
  vpHomogeneousMatrix bMf3 = directKinematics(q,3,fksolver);
  vpHomogeneousMatrix bMf4 = directKinematics(q,4,fksolver); //End effector, seria lo mismo poner -1
  vpHomogeneousMatrix bMf5 = directKinematics(q,5,fksolver);

  v.addTransform(bMf1, "base", "Slew", "1");
  v.addTransform(bMf2, "base", "Shoulder", "2");
  v.addTransform(bMf3, "base", "Elbow", "3");
  v.addTransform(bMf4, "base", "End_effector", "4");*/

  while(nh.ok()){

    v.publish();
    rate.sleep();
    ros::spinOnce();

  }
  return 1;

}
