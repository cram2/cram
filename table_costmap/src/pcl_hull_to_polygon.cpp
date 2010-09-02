
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/subscriber.h>
#include <geometry_msgs/PolygonStamped.h>

typedef pcl_ros::Subscriber<pcl::PointXYZ> PointCloudSubscriber;

void pointCloudToPolygonCb(const PointCloudSubscriber::PointCloud::ConstPtr &cloud, ros::Publisher &polygon_pub)
{
  geometry_msgs::PolygonStamped polygon;

  polygon.header = cloud->header;
  BOOST_FOREACH(const pcl::PointXYZ &pt, cloud->points)
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
  PointCloudSubscriber pcl_subscription = PointCloudSubscriber(
    nh, "input", 10, boost::bind(&pointCloudToPolygonCb, _1, polygon_pub));

  ros::spin();
}
