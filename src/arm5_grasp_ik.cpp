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
	  //As known errors often arrise, ignore
      //ROS_ERROR("%s",ex.what());
    } 
    if(found){   
      //IK: Check reachabillity...
      vpHomogeneousMatrix bMg=VispUtils::vispHomogFromTfTransform(tf::Transform(transform));
 
      vpColVector final_joints(5);
      final_joints=robot.armIK(bMg);
      vpHomogeneousMatrix bMg_fk;
      vpColVector final_joints2(5);
      final_joints2[0]=final_joints[0];
      final_joints2[1]=final_joints[1];
      final_joints2[2]=final_joints[2];
      final_joints2[3]=0;
      final_joints2[4]=0;
      bMg_fk=robot.directKinematics(final_joints2);
      //std::cout << "Reach to: " << bMg_fk << std::endl;
      //std::cout << "Desired: " << bMg << std::endl;
      std::cout << "Distance to desired cMg: " << (bMg.column(4)-bMg_fk.column(4)).euclideanNorm() << std::endl; 
      std::cout << "Reachable position joints: " << std::endl << final_joints2 << std::endl;    
    //If found or if distance < threshold
    //Here may -> Sleep and execute    
    //Publish FK of foun joint config
    tf::StampedTransform fk(VispUtils::tfTransFromVispHomog(bMg_fk), ros::Time::now(), "/kinematic_base", "/reachable_cMg");
    broadcaster->sendTransform(fk);
    
    }
    
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
