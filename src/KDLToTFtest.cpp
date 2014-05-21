/*
 * Transform.cpp
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

/*
   std::map<std::string, double> getFullJointMap()
   {
   int getNumberOfJoints=4;
   std::map<std::string, double> map;
/*for(int i=0;i<getNumberOfJoints;i++)
{
map[names[i]]=q[mimic[i].joint];

}* /
map["Shoulder"]=0.0;
map["Slew"]=1.0;
map["Elbow"]=0.0;

return map;
}*/

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

  vpColVector q(5);
  q[0]=atof(argv[1]);
  q[1]=atof(argv[2]);
  q[2]=atof(argv[3]);
  q[3]=atof(argv[4]);
  q[4]=atof(argv[5]);
  
  ROS_INFO("I got through this");sleep(1);

  //Definition of a kinematic chain & add segments to the chain
  KDL::Chain chain;

  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.08052,  M_PI_2,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.44278, 0.0 ,  0.0    , 7.98*M_PI/180     )));//7.98
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.083 ,  M_PI_2,  0.0    , 113*M_PI/180     )));//127.02
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0,  0.49938    , 0.0     )));


  KDL::ChainFkSolverPos_recursive fksolver (chain);
  //= new KDL::ChainFkSolverPos_recursive(chain);

  //Segments start in 0?
  vpHomogeneousMatrix bMf0 = directKinematics(q,0,fksolver);
  vpHomogeneousMatrix bMf1 = directKinematics(q,1,fksolver);
  vpHomogeneousMatrix bMf2 = directKinematics(q,2,fksolver);
  vpHomogeneousMatrix bMf3 = directKinematics(q,3,fksolver);
  vpHomogeneousMatrix bMf4 = directKinematics(q,4,fksolver);
  vpHomogeneousMatrix bMf5 = directKinematics(q,5,fksolver);

  vpHomogeneousMatrix bMeef = directKinematics(q,-1,fksolver);

  KDL::Tree tree;
  tree.addChain(chain, "root");

  ROS_INFO("Loaded tree, %d segments, %d joints", tree.getNrOfSegments(), tree.getNrOfJoints());

  v.addTransform(bMf0, "world", "base", "s");//No transforma, es 0 0 0 0 0 0
  v.addTransform(bMf1, "base", "Slew", "1");
  v.addTransform(bMf2, "Slew", "Shoulder", "2");
  v.addTransform(bMf3, "Shoulder", "Elbow", "3");
  v.addTransform(bMf4, "Elbow", "j4", "4");
  v.addTransform(bMf5, "j4", "j5", "5");

  v.addTransform(bMeef, "base", "end_effector", "6");



  while(nh.ok()){



    //publishTransforms(const std::map<std::string, double>& joint_positions,       ros::Time);
    //robot.publishTransforms(getFullJointMap(), ros::Time::now(), "dvd");
    v.publish();
    rate.sleep();
    ros::spinOnce();

  }
  return 0;

}
