/*#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "cloud_frame"));
    r.sleep();
  }
}
*/

 #include <ros/ros.h>
 #include <tf/transform_broadcaster.h>
 #include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

geometry_msgs::Pose posicao;
geometry_msgs::Quaternion orientacao;

 void MsgCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {

	posicao = msg->pose.pose;
    orientacao = posicao.orientation;
    ROS_INFO("foi");

 }



 int main(int argc, char** argv){
   ros::init(argc, argv, "my_tf_broadcaster");
   ros::NodeHandle node;

   ros::Subscriber subscriber = node.subscribe("RosAria/pose", 1000, MsgCallback);


   ros::Rate rate(10.0);
      while (node.ok()){
    	  tf::TransformBroadcaster br2;
    	  tf::Transform transform2;

    	  transform2.setOrigin( tf::Vector3(0.1, 0.0, 0.2) );
    	  transform2.setRotation( tf::Quaternion(-1, 0, 0, 1) );
    	  br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "cloud_frame"));

    	  tf::TransformBroadcaster br;
    	  tf::Transform transform;

    	  transform.setOrigin(tf::Vector3(posicao.position.x , posicao.position.y , posicao.position.z));
    	  transform.setRotation(tf::Quaternion(orientacao.x,orientacao.y,orientacao.z,orientacao.w));
    	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "map"));




        rate.sleep();
      }

   ros::spin();

   return 0;
 };
