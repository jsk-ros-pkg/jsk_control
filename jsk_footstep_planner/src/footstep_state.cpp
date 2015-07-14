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

#include "jsk_footstep_planner/footstep_state.h"

#include <eigen_conversions/eigen_msg.h>

#include <jsk_pcl_ros/pcl_conversion_util.h>
#include <jsk_pcl_ros/geo_util.h>

namespace jsk_footstep_planner
{
  jsk_footstep_msgs::Footstep::Ptr
  FootstepState::toROSMsg()
  {
    jsk_footstep_msgs::Footstep::Ptr ret(new jsk_footstep_msgs::Footstep);
    tf::poseEigenToMsg(pose_, ret->pose);
    ret->dimensions.x = dimensions_[0];
    ret->dimensions.y = dimensions_[1];
    ret->dimensions.z = dimensions_[2];
    ret->leg = leg_;
    return ret;
  }

  pcl::PointIndices::Ptr
  FootstepState::cropPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
  {
    pcl::CropBox<pcl::PointNormal> crop_box(false);
    Eigen::Vector4f max_points(dimensions_[0]/2,
                               dimensions_[1]/2,
                               1000.0,
                               0);
    Eigen::Vector4f min_points(-dimensions_[0]/2,
                               -dimensions_[1]/2,
                               -1000.0,
                               0);
    crop_box.setInputCloud(cloud);
    crop_box.setMax(max_points);
    crop_box.setMin(min_points);
    float roll, pitch, yaw;
    pcl::getEulerAngles(pose_, roll, pitch, yaw);
    crop_box.setTranslation(pose_.translation());
    crop_box.setRotation(Eigen::Vector3f(roll, pitch, yaw));
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    crop_box.filter(indices->indices);
    return indices;
  }
  
  FootstepState::Ptr
  FootstepState::projectToCloud(pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                                pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                                const Eigen::Vector3f& z,
                                unsigned int& error_state,
                                double outlier_threshold,
                                int max_iterations,
                                int min_inliers)
  {
    // TODO: z is ignored
    // extract candidate points
    pcl::PointIndices::Ptr indices = cropPointCloud(cloud);
    // estimate plane with ransac
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointNormal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setRadiusLimits(0.3, std::numeric_limits<double>::max ());
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(outlier_threshold);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setInputCloud(cloud);
    seg.setIndices(indices);
    seg.setMaxIterations(max_iterations);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      error_state = projection_state::no_plane;
      return FootstepState::Ptr();
    }
    else if (inliers->indices.size() < min_inliers) {
      error_state = projection_state::no_enough_inliers;
      return FootstepState::Ptr();
    }
    else {
      error_state = projection_state::success;
      jsk_pcl_ros::Plane plane(coefficients->values);
      if (!plane.isSameDirection(z)) {
        plane = plane.flip();
      }
      Eigen::Vector3f n = plane.getNormal();
      Eigen::Quaternionf rot;
      rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), n);
      Eigen::Quaternionf new_rot(pose_.rotation() * rot);
      Eigen::Vector3f p(pose_.translation());
      double alpha = (- plane.getD() - n.dot(p)) / (n.dot(z));
      Eigen::Vector3f q = p + alpha * z;
      Eigen::Affine3f new_pose = Eigen::Translation3f(q) * new_rot;
      return FootstepState::Ptr(new FootstepState(leg_, new_pose, dimensions_));
    }
  }
}
