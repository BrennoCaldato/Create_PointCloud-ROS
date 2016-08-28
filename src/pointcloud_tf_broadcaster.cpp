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
 
 int main(int argc, char** argv){
   ros::init(argc, argv, "my_tf_broadcaster");
   ros::NodeHandle node;
 
   tf::TransformBroadcaster br;
   tf::Transform transform;
 
   ros::Rate rate(10.0);
   while (node.ok()){
     transform.setOrigin( tf::Vector3(0.1, 0.0, 0.2) );
     transform.setRotation( tf::Quaternion(-1, 0, 0, 1) );
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "cloud_frame"));
     rate.sleep();
   }
   return 0;
 };
