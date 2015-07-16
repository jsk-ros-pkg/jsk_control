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
  FootstepState::cropPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                                pcl::search::Octree<pcl::PointNormal>& tree)
  {
    pcl::PointNormal center;
    center.getVector3fMap() = Eigen::Vector3f(pose_.translation());
    float r = 0.2;
    pcl::PointIndices::Ptr near_indices(new pcl::PointIndices);
    std::vector<float> distances;
    tree.radiusSearch(center, r, near_indices->indices, distances);

    // Project vertices into 2d
    Eigen::Vector3f z(0, 0, 1);
    Eigen::Vector3f a = pose_ * Eigen::Vector3f(dimensions_[0]/2, dimensions_[1]/2, 0);
    Eigen::Vector3f b = pose_ * Eigen::Vector3f(-dimensions_[0]/2, dimensions_[1]/2, 0);
    Eigen::Vector3f c = pose_ * Eigen::Vector3f(-dimensions_[0]/2, -dimensions_[1]/2, 0);
    Eigen::Vector3f d = pose_ * Eigen::Vector3f(dimensions_[0]/2, -dimensions_[1]/2, 0);
    Eigen::Vector3f a_2d = a + (- z.dot(a)) * z;
    Eigen::Vector3f b_2d = b + (- z.dot(b)) * z;
    Eigen::Vector3f c_2d = c + (- z.dot(c)) * z;
    Eigen::Vector3f d_2d = d + (- z.dot(d)) * z;

    Eigen::Vector2f a2d(a_2d[0], a_2d[1]);
    Eigen::Vector2f b2d(b_2d[0], b_2d[1]);
    Eigen::Vector2f c2d(c_2d[0], c_2d[1]);
    Eigen::Vector2f d2d(d_2d[0], d_2d[1]);
    std::set<int> set_indices;
    for (size_t i = 0; i < near_indices->indices.size(); i++) {
      size_t index = near_indices->indices[i];
      pcl::PointNormal point = cloud->points[index];
      Eigen::Vector3f point_2d = point.getVector3fMap() + (-z.dot(point.getVector3fMap())) * z;
      Eigen::Vector2f p2d(point_2d[0], point_2d[1]);
      if (cross2d((b2d - a2d), (p2d - a2d)) > 0 &&
          cross2d((c2d - b2d), (p2d - b2d)) > 0 &&
          cross2d((d2d - c2d), (p2d - c2d)) > 0 &&
          cross2d((a2d - d2d), (p2d - d2d)) > 0) {
        set_indices.insert(index);
      }
    }
    pcl::PointIndices::Ptr ret(new pcl::PointIndices);
    ret->indices = std::vector<int>(set_indices.begin(), set_indices.end());
    return ret;
  }
  
  FootstepState::Ptr
  FootstepState::projectToCloud(pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                                pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                                pcl::search::Octree<pcl::PointNormal>& tree_2d,
                                pcl::PointCloud<pcl::PointNormal>::Ptr cloud_2d,
                                const Eigen::Vector3f& z,
                                unsigned int& error_state,
                                double outlier_threshold,
                                int max_iterations,
                                int min_inliers)
  {
    // TODO: z is ignored
    // extract candidate points
    pcl::PointIndices::Ptr indices = cropPointCloud(cloud_2d, tree_2d);
    if (indices->indices.size() < min_inliers) {
      error_state = projection_state::no_enough_inliers;
      return FootstepState::Ptr();
    }
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
      return FootstepState::Ptr(new FootstepState(leg_, new_pose, dimensions_,
                                                  resolution_,
                                                  index_x_,
                                                  index_y_,
                                                  index_yaw_));
    }
  }
}
