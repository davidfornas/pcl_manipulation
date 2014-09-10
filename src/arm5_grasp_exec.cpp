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


//Target joints publisher
ros::Publisher js_pub;

//FK final position
tf::TransformBroadcaster *broadcaster;

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm5_grasp_exec");
  ros::NodeHandle nh;

  ARM5Arm robot(nh, "/uwsim/joint_state", "/uwsim/joint_state_command");

  //js_pub =nh.advertise<sensor_msgs::JointState>("/uwsim/joint_state",1);
  broadcaster = new tf::TransformBroadcaster();

  ros::Rate rate(1);
  tf::TransformListener listener;
  //Create ARM5Robot  
  
  bool found = false;

  tf::StampedTransform transform;//Una vez tiene una trasformación la puede publicar contínuamente aunque ya no la encuentre...
  //Esto me permitirai apagar el grasp planning para ejecutar...
  while(nh.ok()){

    try{
      listener.lookupTransform("/kinematic_base", "/cMg", ros::Time(0), transform);//Si falla espero que no modifique trasnform de ningun modo.
      found = true;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    } 
    if(found){   
      //IK: Check reachabillity...
      vpHomogeneousMatrix bMg=VispUtils::vispHomogFromTfTransform(tf::Transform(transform));
 
      vpColVector final_joints(5);
      final_joints=robot.armIK(bMg);
      vpHomogeneousMatrix bMg_fk=robot.directKinematics(final_joints);
      std::cout << "Distance: " << (bMg.column(3)-bMg_fk.column(3)).euclideanNorm() << std::endl; 
      std::cout << "JOINTS: " << final_joints << std::endl;
    std::cout << "mat: " << bMg_fk << std::endl;
    
    //If found
    //Sleep and execute
    
    
    //FK
    tf::StampedTransform fk(VispUtils::tfTransFromVispHomog(bMg_fk), ros::Time::now(), "/kinematic_base", "/reachable_cMg");
    broadcaster->sendTransform(fk);
    }
    
    rate.sleep();
    ros::spinOnce();

  }
  return 0;

}
