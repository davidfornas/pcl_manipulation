#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//Target pose publisher
ros::Publisher pose_pub, vel_pub;
//What to do...
bool execute=false, stop=false, up=false;
//TF Broadcaster to visualize target pose, could be a marker also.
tf::TransformBroadcaster *broadcaster;


//Command
void command_callback(const std_msgs::String::ConstPtr& comm)
{
  execute=(comm->data=="exec"?true:false);
  up=(comm->data=="up"?true:false);
  stop=(comm->data=="stop"?true:false);        
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grasp_exec");
  ros::NodeHandle nh;

  pose_pub =nh.advertise<geometry_msgs::PoseStamped>("target_position_in",1);
  vel_pub =nh.advertise<geometry_msgs::TwistStamped>("target_velocity_in",1);

  ros::Subscriber comm_sub = nh.subscribe<std_msgs::String>("command",1,command_callback);

  broadcaster = new tf::TransformBroadcaster();

  ros::Rate rate(15);

  tf::TransformListener listener;

  tf::Vector3 p2(0,0,0);
  tf::Quaternion tf_q2; tf_q2.setEuler(0, -1.57, 0);
  tf::Transform pose2;  pose2.setOrigin(p2);   pose2.setRotation(tf_q2);

  bool exception=true;
  tf::StampedTransform transform;//Una vez tiene una trasformación la puede publicar contínuamente aunque ya no la encuentre...
  //Esto me permitirai apagar el grasp planning para ejecutar...
  while(nh.ok()){

    tf::StampedTransform t2(pose2, ros::Time::now(), "/grasp_frame", "/grasp");
    broadcaster->sendTransform(t2);
    try{
      listener.lookupTransform("/camera", "/grasp", ros::Time(0), transform);//Si falla espero que no modifique trasnform de ningun modo.
      exception = false;
    }
    catch (tf::TransformException ex){
      //ONLY IF NEEDED: ROS_ERROR("%s",ex.what());
      if(!exception){
        tf::StampedTransform t3(transform, ros::Time::now(), "/camera", "/grasp");
        broadcaster->sendTransform(t3);
      }//Si se ha leido una vez al menos...La publica él...
    }
    if(!exception && !stop){
      if(execute){

        //From cMg to cartesian Twist
        geometry_msgs::TwistStamped target_vel;
        target_vel.header.frame_id = "/grasp";//Twist is calculetd with respect to the camera

        target_vel.twist.linear.x = 0;
        target_vel.twist.linear.y = 0;
        target_vel.twist.linear.z = 0.02;

        double x,y,z;
        transform.getBasis().getRPY(x,y,z);

        target_vel.twist.angular.x = 0;
        target_vel.twist.angular.y = 0;
        target_vel.twist.angular.z = 0;
        target_vel.header.stamp = ros::Time::now();

        vel_pub.publish(target_vel);execute=false;

      }else{

        if(up){
          //From cMg to cartesian Pose
          geometry_msgs::PoseStamped target_pose;
          target_pose.header.frame_id = "/grasp";//Pose is calculetd with respect to the camera

          target_pose.pose.position.x = 0.2;
          target_pose.pose.position.y = 0;
          target_pose.pose.position.z = -0,2;
          target_pose.pose.orientation.x = 0;
          target_pose.pose.orientation.y = 0;
          target_pose.pose.orientation.z = 0;
          target_pose.pose.orientation.w = 1;
          target_pose.header.stamp = ros::Time::now();

          pose_pub.publish(target_pose);

        }else{
          //From cMg to cartesian Pose
          geometry_msgs::PoseStamped target_pose;
          target_pose.header.frame_id = "/camera";//Pose is calculetd with respect to the camera

          target_pose.pose.position.x = transform.getOrigin().x();
          target_pose.pose.position.y = transform.getOrigin().y();
          target_pose.pose.position.z = transform.getOrigin().z();
          target_pose.pose.orientation.x = transform.getRotation().x();
          target_pose.pose.orientation.y = transform.getRotation().y();
          target_pose.pose.orientation.z = transform.getRotation().z();
          target_pose.pose.orientation.w = transform.getRotation().w();
          target_pose.header.stamp = ros::Time::now();

          pose_pub.publish(target_pose);
        }
      }
    }//End if exception
    rate.sleep();
    ros::spinOnce();

  }
  return 0;

}
