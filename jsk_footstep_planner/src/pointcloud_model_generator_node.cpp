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
#include <ros/ros.h>
#include "jsk_footstep_planner/pointcloud_model_generator.h"
#include <jsk_footstep_planner/PointCloudModelGeneratorConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_pcl_ros/pcl_conversion_util.h>
#include <dynamic_reconfigure/server.h>

using namespace jsk_footstep_planner;

ros::Publisher pub_cloud;

void reconfigureCallback(PointCloudModelGeneratorConfig &config, uint32_t level)
{
  PointCloudModelGenerator gen;
  pcl::PointCloud<pcl::PointNormal> cloud;
  gen.generate(config.model, cloud);
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(cloud, ros_cloud);
  ros_cloud.header.frame_id = "odom";
  ros_cloud.header.stamp = ros::Time::now();
  pub_cloud.publish(ros_cloud);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_model_generator_node");
  ros::NodeHandle pnh("~");
  pub_cloud = pnh.advertise<sensor_msgs::PointCloud2>("output", 1, true);
  dynamic_reconfigure::Server<PointCloudModelGeneratorConfig> server;
  server.setCallback(boost::bind(&reconfigureCallback, _1, _2));
  ros::spin();
  return 0;
}
