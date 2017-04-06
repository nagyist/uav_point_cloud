#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

ros::Subscriber front_scan_sub, right_scan_sub, back_scan_sub, left_scan_sub;
ros::Publisher front_point_cloud_pub, right_point_cloud_pub, back_point_cloud_pub, left_point_cloud_pub;
laser_geometry::LaserProjection projector_;

void frontScanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;
  projector_.projectLaser(*scan_in, cloud);

  front_point_cloud_pub.publish( cloud );
}

void rightScanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;
  projector_.projectLaser(*scan_in, cloud);

  right_point_cloud_pub.publish( cloud );
}

void backScanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;
  projector_.projectLaser(*scan_in, cloud);

  back_point_cloud_pub.publish( cloud );
}

void leftScanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;
  projector_.projectLaser(*scan_in, cloud);

  left_point_cloud_pub.publish( cloud );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conversor_node");
    ros::NodeHandle nh;
    
    front_scan_sub = nh.subscribe("/lidar_front/scan", 1, frontScanCallback);
    right_scan_sub = nh.subscribe("/lidar_right/scan", 1, rightScanCallback);
    back_scan_sub = nh.subscribe("/lidar_back/scan", 1, backScanCallback);
    left_scan_sub = nh.subscribe("/lidar_left/scan", 1, leftScanCallback);
    
    front_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/lidar_front/point_cloud", 10);
    right_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/lidar_right/point_cloud", 10);
    back_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/lidar_back/point_cloud", 10);
    left_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/lidar_left/point_cloud", 10);
    
    ros::spin();
	return 0;
}
