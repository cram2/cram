
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

void pointCloudToPolygonCb(const sensor_msgs::PointCloud2::ConstPtr &cloud_raw, ros::Publisher &polygon_pub)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_raw, cloud);
  
  geometry_msgs::PolygonStamped polygon;

  polygon.header = cloud.header;
  BOOST_FOREACH(const pcl::PointXYZ &pt, cloud.points)
  {
    geometry_msgs::Point32 out_pt;
    out_pt.x = pt.x;
    out_pt.y = pt.y;
    out_pt.z = pt.z;
    polygon.polygon.points.push_back(out_pt);
  }

  polygon_pub.publish(polygon);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pcl_hull_to_polygon");

  ros::NodeHandle nh("~");

  ros::Publisher polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("output", 10);
  ros::Subscriber pcl_subscription = nh.subscribe<sensor_msgs::PointCloud2>(
    "input", 10, boost::bind(&pointCloudToPolygonCb, _1, polygon_pub));

  ros::spin();
}
