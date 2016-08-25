// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <jsk_footstep_msgs/FootstepArray.h>
#include "jsk_footstep_planner/footstep_state.h"
#include <interactive_markers/tools.h>
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>

//#include <jsk_interactive_marker/interactive_marker_helpers.h>
using namespace jsk_footstep_planner;

// globals
ros::Publisher pub_footstep;
ros::Publisher pub_projected_footstep;
ros::Publisher pub_cloud;
FootstepState::Ptr original_footstep;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
pcl::KdTreeFLANN<pcl::PointNormal> tree;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud2d;
pcl::search::Octree<pcl::PointNormal> tree2d(0.1);
ANNGrid::Ptr grid_search;
FootstepParameters parameters;

jsk_footstep_msgs::FootstepArray footstepToFootstepArray(
  jsk_footstep_msgs::Footstep msg)
{
  jsk_footstep_msgs::FootstepArray array_msg;
  array_msg.header.frame_id = "odom";
  array_msg.header.stamp = ros::Time::now();
  array_msg.footsteps.push_back(msg);
  return array_msg;
}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  Eigen::Affine3f new_pose;
  tf::poseMsgToEigen(feedback->pose, new_pose);
  original_footstep->setPose(new_pose);
  jsk_footstep_msgs::FootstepArray msg = footstepToFootstepArray(*(original_footstep->toROSMsg()));
  pub_footstep.publish(msg);
  parameters.plane_estimation_outlier_threshold = 0.05;
  parameters.plane_estimation_max_iterations    = 100;
  parameters.plane_estimation_min_inliers       = 10;
  unsigned int error_state;
  FootstepState::Ptr projected_footstep = original_footstep->projectToCloud(
    tree,
    cloud,
    grid_search,
    tree2d,
    cloud2d,
    Eigen::Vector3f(0, 0, 1),
    error_state,
    parameters);
  if (projected_footstep) {
    jsk_footstep_msgs::FootstepArray msg2 = footstepToFootstepArray(*(projected_footstep->toROSMsg()));
    pub_projected_footstep.publish(msg2);
  }
  else {
    ROS_ERROR("error state: %u" , error_state);
  }
}

pcl::PointCloud<pcl::PointNormal>::Ptr
generateCloud()
{
  pcl::PointCloud<pcl::PointNormal>::Ptr gen_cloud(new pcl::PointCloud<pcl::PointNormal>);
  for (double y = -0.5; y < 0.5; y = y + 0.01) {
    for (double x = 0.0; x < 0.5; x = x + 0.01) {
      pcl::PointNormal p;
      p.x = x;
      p.y = y;
      gen_cloud->points.push_back(p);
    }
    for (double x = 0.5; x < 1.0; x = x + 0.01) {
      pcl::PointNormal p;
      p.x = x;
      p.y = y;
      p.z = x - 0.5;
      gen_cloud->points.push_back(p);
    }
    for (double x = 1.0; x < 1.5; x = x + 0.01) {
      pcl::PointNormal p;
      p.x = x;
      p.y = y;
      p.z = 0.5;
      gen_cloud->points.push_back(p);
    }
    for (double x = 1.5; x < 2.0; x = x + 0.01) {
      pcl::PointNormal p;
      p.x = x;
      p.y = y;
      p.z = -x + 2.0;
      gen_cloud->points.push_back(p);
    }
    for (double x = 2.0; x < M_PI; x = x + 0.01) {
      pcl::PointNormal p;
      p.x = x;
      p.y = y;
      gen_cloud->points.push_back(p);
    }
    for (double x = M_PI; x < 2.0 * M_PI; x = x + 0.01) {
      pcl::PointNormal p;
      p.x = x;
      p.y = y;
      p.z = std::abs(sin(2.0 * x));
      gen_cloud->points.push_back(p);
    }

  }
  return gen_cloud;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footstep_projection_demo");
  ros::NodeHandle nh("~");
  pub_footstep
    = nh.advertise<jsk_footstep_msgs::FootstepArray>("original", 1);
  pub_projected_footstep
    = nh.advertise<jsk_footstep_msgs::FootstepArray>("projected", 1);
  pub_cloud
    = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1, true);
  
  // generate pointcloud
  cloud = generateCloud();
  cloud2d.reset(new pcl::PointCloud<pcl::PointNormal>);
  tree.setInputCloud(cloud);
  grid_search.reset(new ANNGrid(0.1));
  grid_search->build(*cloud);
  pcl::ProjectInliers<pcl::PointNormal> proj;
  pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients);
  
  coef->values.resize(4);
  coef->values[2] = 1.0;
  proj.setInputCloud(cloud);
  proj.setModelCoefficients(coef);
  proj.filter(*cloud2d);
  tree2d.setInputCloud(cloud2d);
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header.frame_id = "odom";
  ros_cloud.header.stamp = ros::Time::now();
  pub_cloud.publish(ros_cloud);
  original_footstep.reset(new FootstepState(jsk_footstep_msgs::Footstep::LEFT,
                                            Eigen::Affine3f::Identity(),
                                            Eigen::Vector3f(0.2, 0.1, 0.00001),
                                            Eigen::Vector3f(0.05, 0.05, 0.8)));
  jsk_footstep_msgs::FootstepArray msg = footstepToFootstepArray(*(original_footstep->toROSMsg()));
  pub_footstep.publish(msg);
  interactive_markers::InteractiveMarkerServer server("footstep_projection_demo");
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/odom";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "footstep marker";

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  server.insert(int_marker, &processFeedback);
  server.applyChanges();
  ros::spin();
}
