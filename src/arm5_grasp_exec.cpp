/** 
 * Grasp execution utility. Moves the ARM5e to the desired cMg
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
void openGripper(double velocity, double aperture, double current){
	vpColVector vel(5), current_joints;
	vel=0;
	vel[4]=velocity;
	robot_->getJointValues(current_joints);
	if(velocity>0){
		while(current_joints[4]<aperture && ros::ok() && robot_->getCurrent()<current ){
			robot_->setJointVelocity(vel);
			ros::spinOnce();
			robot_->getJointValues(current_joints);
		}
	}
	else{
		while(current_joints[4]>aperture && ros::ok() && robot_->getCurrent()<current ){
			robot_->setJointVelocity(vel);
			ros::spinOnce();
			robot_->getJointValues(current_joints);
		}
	}
}

///Moved from GRASPER
///Reach a position with respect to a given 
	void reachPosition(vpHomogeneousMatrix cMgoal, vpHomogeneousMatrix bMc){
		vpHomogeneousMatrix bMe, cMe;
		//joint_offset_->get_bMc(bMc);
		
		robot_->getPosition(bMe);
		cMe=bMc.inverse()*bMe;
		while((cMe.column(4)-cMgoal.column(4)).euclideanNorm()>0.02 && ros::ok()){
			std::cout<<"Error: "<<(cMe.column(4)-cMgoal.column(4)).euclideanNorm()<<std::endl;
			vpColVector xdot(6);
			xdot=0;
			vpHomogeneousMatrix eMgoal=cMe.inverse()*cMgoal;
			xdot[0]=eMgoal[0][3]*0.6;
			xdot[1]=eMgoal[1][3]*0.6;
			xdot[2]=eMgoal[2][3]*0.6;
			robot_->setCartesianVelocity(xdot);
			ros::spinOnce();

			robot_->getPosition(bMe);
			cMe=bMc.inverse()*bMe;
		}
}





//Target joints publisher
//ros::Publisher js_pub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm5_grasp_exec");
  ros::NodeHandle nh;

  robot_ = new ARM5Arm(nh, "/uwsim/joint_state", "/uwsim/joint_state_command");

  //js_pub =nh.advertise<sensor_msgs::JointState>("/uwsim/joint_state",1);
  //broadcaster = new tf::TransformBroadcaster();

  ros::Rate rate(1);
  tf::TransformListener listener;
  //Create ARM5Robot  
  
  tf::StampedTransform transform;
  //bMc
  bool bMc_found = false;
  //Buscar la posición deseada...
  while(nh.ok()&&!bMc_found){

    try{
      listener.lookupTransform("/kinematic_base", "/stereo", ros::Time(0), transform);
      bMc_found = true;
    }
    catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
    } 
    rate.sleep();
    ros::spinOnce();
  }
  vpHomogeneousMatrix bMc=VispUtils::vispHomogFromTfTransform(tf::Transform(transform));
    
  bool cMgoal_found = false;
  //Buscar la posición deseada...
  while(nh.ok()&&!cMgoal_found){

    try{
      listener.lookupTransform("/stereo", "/reachable_cMg", ros::Time(0), transform);//Si falla espero que no modifique trasnform de ningun modo.
      cMgoal_found = true;
    }
    catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
    } 
    rate.sleep();
    ros::spinOnce();
   }
   vpHomogeneousMatrix cMgoal=VispUtils::vispHomogFromTfTransform(tf::Transform(transform));
  
   reachPosition(cMgoal, bMc);
  
  /*
   //Mover a la posición en velocidad
   //Podría hacerlo en velocidad por diferencia de articulaciones
   // o como en grasper...
   while(nh.ok()){
	   
      //IK: Check reachabillity...
      vpHomogeneousMatrix bMg=VispUtils::vispHomogFromTfTransform(tf::Transform(transform));
 
      vpColVector final_joints(5);
      final_joints=robot_->armIK(bMg);
      vpHomogeneousMatrix bMg_fk;
      vpColVector final_joints2(5);
      final_joints2[0]=final_joints[0];
      final_joints2[1]=final_joints[1];
      final_joints2[2]=final_joints[2];
      final_joints2[3]=0;
      final_joints2[4]=0;
      bMg_fk=robot_->directKinematics(final_joints2);
      //std::cout << "Reach to: " << bMg_fk << std::endl;
      //std::cout << "Desired: " << bMg << std::endl;
      std::cout << "Distance to desired cMg: " << (bMg.column(4)-bMg_fk.column(4)).euclideanNorm() << std::endl; 
      std::cout << "Reachable position joints: " << std::endl << final_joints2 << std::endl;    
    //If found or if distance < threshold
    //Here may -> Sleep and execute    
    //Publish FK of foun joint config
    tf::StampedTransform fk(VispUtils::tfTransFromVispHomog(bMg_fk), ros::Time::now(), "/kinematic_base", "/reachable_cMg");
    //broadcaster->sendTransform(fk);
        rate.sleep();
    ros::spinOnce();
    
    ///v
    }*/
    
    rate.sleep();
    ros::spinOnce();
  
  return 0;
}
