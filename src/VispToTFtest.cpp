/*
 * Test program to VispUtils on MAR/perception
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#include <mar_perception/VispUtils.h>
#include <visp/vpHomogeneousMatrix.h>
#include <stdlib.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "test_visp_to_tf");
  ros::NodeHandle nh;
  vpHomogeneousMatrix cMb(0,0.5,0,0,0,0), cMb2(0,0.5,-0.5,0,0,0), cMb3(0,0,-0.5,1.57,1.57,0), cMb4(0,0,0.5,0,0,0);
  //(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]));
  
  //VispToTF test
  VispToTF v(cMb, "world", "zero");
  v.addTransform(cMb2, "world", "disp","1");//No se está publicando
  v.addTransform(cMb3, "world", "rotated","2");
  v.addTransform(cMb3*cMb4, "world", "rotated_and_disp","6");
  
  //v.resetTransform(cMb4, "9");
  v.print();
  //Remove: v.removeTransform("1");
  
  ros::Rate rate(15);
  
  //VispToMarker test  
  //MarkerPublisher m(cMb2, "other_thing", "pose_marker", nh);
  ///tf::Vector vvv(0,0.5,0);
  ///tf::Transform ttt;
  ///MarkerPublisher m(pose, "other_thing", "pose_marker", nh);
  
  
  
  while(nh.ok()){

    v.publish();
    ///m.publish();
    
    rate.sleep();
    ros::spinOnce();

  }
  return 0;

}
