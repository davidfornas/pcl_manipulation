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

//Target joints publisher
ros::Publisher js_pub;

//FK final position
tf::TransformBroadcaster *broadcaster;

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm5_grasp_exec");
  ros::NodeHandle nh;

  js_pub =nh.advertise<sensor_msgs::JointState>("/uwsim_joint_state",1);
  broadcaster = new tf::TransformBroadcaster();

  ros::Rate rate(1);
  tf::TransformListener listener;

  //Create ARM5Robot
  
  


  tf::StampedTransform transform;//Una vez tiene una trasformación la puede publicar contínuamente aunque ya no la encuentre...
  //Esto me permitirai apagar el grasp planning para ejecutar...
  while(nh.ok()){

    try{
      listener.lookupTransform("/kinematic_base", "/cMg", ros::Time(0), transform);//Si falla espero que no modifique trasnform de ningun modo.
      exception = false;
    }
    catch (tf::TransformException ex){
      //ONLY IF NEEDED: ROS_ERROR("%s",ex.what());
    }    
    //IK: Check reachabillity...
    
    
    
    //If found
    //Sleep and execute
    
    
    //FK
    tf::StampedTransform fk(transform, ros::Time::now(), "/kinematic_base", "/fk_cMg");
    broadcaster->sendTransform(fk);
    
    rate.sleep();
    ros::spinOnce();

  }
  return 0;

}
