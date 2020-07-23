#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Header.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetModelStateRequest.h"
#include "gazebo_msgs/GetModelStateResponse.h"

#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_pub");
  ros::NodeHandle n;
  
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/my_odom", 1000); 
  
  ros::ServiceClient get_model_srv = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  nav_msgs::Odometry odom;
  std_msgs::Header header;
  header.frame_id="/odom";
  
  gazebo_msgs::GetModelStateRequest model;
  model.model_name="centauro";
  gazebo_msgs::GetModelStateResponse result;
 
  ros::Rate loop_rate(10);
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  
  while (ros::ok())
  {
    get_model_srv.call(model,result);   
    odom.pose.pose= result.pose;
    odom.twist.twist = result.twist;
    
    header.stamp = ros::Time::now();
    odom.header = header;
    
    transform.setOrigin( tf::Vector3(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z) );
    //std:: cout << odom.pose.pose.position.x << " " << odom.pose.pose.position.y << " " << odom.pose.pose.position.z << std::endl;
    q.setW(odom.pose.pose.orientation.w);
    q.setX(odom.pose.pose.orientation.x);
    q.setY(odom.pose.pose.orientation.y);
    q.setZ(odom.pose.pose.orientation.z);
    transform.setRotation(q);
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", ""));
    
    odom_pub.publish(odom);
    

    
    ros::spinOnce();
  }
  
  return 0;
}
