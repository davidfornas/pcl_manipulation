/*
 * Transform.cpp
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#include <mar_perception/VispToTF.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <visp/vpHomogeneousMatrix.h>
#include <stdlib.h>

#include <robot_state_publisher/robot_state_publisher.h>

using namespace KDL;

std::map<std::string, double> getFullJointMap()
  {
	int getNumberOfJoints=4;
    std::map<std::string, double> map;
    /*for(int i=0;i<getNumberOfJoints;i++)
    {
      map[names[i]]=q[mimic[i].joint];

    }*/
    map["Shoulder"]=0.0;
    map["Slew"]=1.0;
    map["Elbow"]=0.0;

    return map;
  }



int main(int argc, char **argv) {
  ros::init(argc, argv, "test_tf");
  ros::NodeHandle nh;
  vpHomogeneousMatrix cMb(0,0,0,0,0,0);//(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]));
  
  VispToTF v(cMb, "world", "tony");
  ros::Rate rate(15);

  //Definition of a kinematic chain & add segments to the chain
  KDL::Chain chain;
  chain.addSegment(Segment("Shoulder",Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
  chain.addSegment(Segment("Slew", Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
  chain.addSegment(Segment("Elbow",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
  chain.addSegment(Segment(Joint(Joint::RotZ)));
  chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
  chain.addSegment(Segment(Joint(Joint::RotZ)));

  KDL::Tree tree;
  tree.addChain(chain, "root");

  ROS_INFO("Loaded tree, %d segments, %d joints", tree.getNrOfSegments(), tree.getNrOfJoints());


  robot_state_publisher::RobotStatePublisher robot(tree);//const KDL::Tree& tree);
  //RobotStatePublisher(const KDL::Tree& tree);
  // publish moving joints
   //void publishTransforms(const std::map<std::string, double>& joint_positions,
     //                     const ros::Time& time);
   // publish fixed joints
  // void publishFixedTransforms();
  ROS_INFO("Info : %f  ", getFullJointMap()["Shoulder"]);


  while(nh.ok()){



	//publishTransforms(const std::map<std::string, double>& joint_positions,       ros::Time);
	robot.publishTransforms(getFullJointMap(), ros::Time::now(), "dvd");
	//v.publish();
    rate.sleep();
    ros::spinOnce();

  }
  return 0;

}
