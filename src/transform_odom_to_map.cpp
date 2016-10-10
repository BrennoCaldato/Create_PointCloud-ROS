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

 void MsgCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {
    geometry_msgs::Pose posicao;
    geometry_msgs::Quaternion orientacao;

	  posicao = msg->pose.pose;
    orientacao = posicao.orientation;
    ROS_INFO("foi");

    tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(posicao.position.x , posicao.position.y , posicao.position.z));
    transform.setRotation(tf::Quaternion(orientacao.x,orientacao.y,orientacao.z,orientacao.w));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));


 }



 int main(int argc, char** argv){
   ros::init(argc, argv, "my_tf_broadcaster2");
   ros::NodeHandle node;

   ros::Subscriber subscriber = node.subscribe("RosAria/pose", 100, MsgCallback);

   ros::spin();

   return 0;
 };
