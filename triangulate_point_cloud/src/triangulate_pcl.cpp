/**
 * Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors 
 *       may be used to endorse or promote products derived from this software 
 *       without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>

#include <boost/foreach.hpp>

#include <ros/ros.h>

#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/surface/convex_hull.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <triangulate_point_cloud/TriangulatePCL.h>
#include <arm_navigation_msgs/Shape.h>
#include <geometry_msgs/Point.h>


using namespace pcl;
using namespace triangulate_point_cloud;

void reconstructMesh(const PointCloud<PointXYZ>::ConstPtr &cloud,
  PointCloud<PointXYZ> &output_cloud, std::vector<Vertices> &triangles)
{
  boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
  indices->resize(cloud->points.size ());
  for (size_t i = 0; i < indices->size (); ++i) { (*indices)[i] = i; }

  pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
  tree->setInputCloud(cloud);

  PointCloud<PointXYZ>::Ptr mls_points(new PointCloud<PointXYZ>);
  PointCloud<PointNormal>::Ptr mls_normals(new PointCloud<PointNormal>);
  MovingLeastSquares<PointXYZ, PointNormal> mls;

  mls.setInputCloud(cloud);
  mls.setIndices(indices);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);
  
  mls.process(*mls_normals);
  
  ConvexHull<PointXYZ> ch;
  
  ch.setInputCloud(mls_points);
  ch.reconstruct(output_cloud, triangles);
}

template<typename T>
void toPoint(const T &in, geometry_msgs::Point &out)
{
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

template<typename T>
void polygonMeshToShapeMsg(const PointCloud<T> &points,
  const std::vector<Vertices> &triangles,
  arm_navigation_msgs::Shape &shape)
{
  shape.type = arm_navigation_msgs::Shape::MESH;
  shape.vertices.resize(points.points.size());
  for(size_t i=0; i<points.points.size(); i++)
    toPoint(points.points[i], shape.vertices[i]);

  ROS_INFO("Found %ld polygons", triangles.size());
  BOOST_FOREACH(const pcl::Vertices polygon, triangles)
  {
    if(polygon.vertices.size() < 3)
    {
      ROS_WARN("Not enough points in polygon. Ignoring it.");
      continue;
    }
      
    shape.triangles.push_back(polygon.vertices[0]);
    shape.triangles.push_back(polygon.vertices[1]);
    shape.triangles.push_back(polygon.vertices[2]);
  }
}

bool onTriangulatePcl(TriangulatePCL::Request &req, TriangulatePCL::Response &res)
{
  ROS_INFO("Service request received");
  
  sensor_msgs::PointCloud2 cloud_raw;
  sensor_msgs::convertPointCloudToPointCloud2(req.points, cloud_raw);
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ> out_cloud;
  fromROSMsg(cloud_raw, *cloud);

  std::vector<Vertices> triangles;

  ROS_INFO("Triangulating");
  reconstructMesh(cloud, out_cloud, triangles);
  ROS_INFO("Triangulation done");

  ROS_INFO("Converting to shape message");
  polygonMeshToShapeMsg(out_cloud, triangles, res.mesh);

  ROS_INFO("Service processing done");
  
  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "triangulate_point_cloud");
  ros::NodeHandle nh("~");
  
  ros::ServiceServer service = nh.advertiseService("triangulate", &onTriangulatePcl);
  ROS_INFO("Triangulation service running");
  ros::spin();
  return 0;
}
