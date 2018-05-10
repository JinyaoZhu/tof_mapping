#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub_pc;


tf::Transform T_0;
tf::Transform T_1;
tf::Transform T_2;
tf::Transform T_3;

bool isPoseInit = false;

void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  geometry_msgs::Pose pose = msg->pose.pose;

  static tf::TransformBroadcaster br;
  tf::Transform T_m;

  static bool isInit = false;

  if(!isInit){
    T_0.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
    T_0.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    isInit = true;
    isPoseInit = true;
    return;
  }

  T_m.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
  T_m.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
  T_1 = T_0.inverseTimes(T_m);
  T_3 = T_1*T_2;
  br.sendTransform(tf::StampedTransform(T_1, ros::Time::now(), "world", "tracker"));
  br.sendTransform(tf::StampedTransform(T_3, ros::Time::now(), "world", "cloud"));
}


void pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  sensor_msgs::PointCloud2 out_msg;
  out_msg = *msg;
  out_msg.header.stamp = ros::Time::now();
  if(isPoseInit)
    pub_pc.publish(out_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tof_mapping");

  ros::NodeHandle nh("~");

  T_2.setOrigin( tf::Vector3(0.065,0,0.03) );
  T_2.setRotation(tf::Quaternion(-0.5,0.5,-0.5,0.5));

  pub_pc = nh.advertise<sensor_msgs::PointCloud2>("pointCloud",1000);
//  ros::Subscriber sub_odo = nh.subscribe("/vins_estimator/camera_pose", 1000, pose_callback);
//  ros::Subscriber sub_odo = nh.subscribe("/vive/LHR_08DDEDC9_odom", 1000, pose_callback);
//  ros::Subscriber sub_pc = nh.subscribe("/bta_tof_driver_1/tof_camera/point_cloud_xyz", 1000, pc_callback,ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_odo = nh.subscribe("/pose_in", 1000, pose_callback);
  ros::Subscriber sub_pc = nh.subscribe("/cloud_in", 1000, pc_callback,ros::TransportHints().tcpNoDelay());

  ros::spin();

  return 0;
}

