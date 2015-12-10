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

#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_recognition_utils/geo_util.h>
#include "jsk_footstep_planner/line2d.h"

namespace jsk_footstep_planner
{

  std::string projectStateToString(unsigned int state)
  {
    if (state == projection_state::success) {
      return "success";
    }
    else if (state == projection_state::no_pointcloud) {
      return "no pointcloud";
    }
    else if (state == projection_state::no_enough_support) {
      return "no enough support";
    }
    else if (state == projection_state::no_plane) {
      return "no plane";
    }
    else if (state == projection_state::no_enough_inliers) {
      return "no enough inliers";
    }
    else if (state == projection_state::close_to_success) {
      return "close to success";
    }
    else {
      return "unknown error";
    }
  }
  
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
  FootstepState::cropPointCloudExact(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                                     pcl::PointIndices::Ptr near_indices)
  {
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
    //std::set<int> set_indices;
    pcl::PointIndices::Ptr ret(new pcl::PointIndices);
    ret->indices.reserve(near_indices->indices.size());
    const std::vector<int> near_indices_indices = near_indices->indices;
    for (size_t i = 0; i < near_indices->indices.size(); i++) {
      const int index = near_indices_indices[i];
      const pcl::PointNormal point = cloud->points[index];
      const Eigen::Vector3f ep = point.getVector3fMap();
      const Eigen::Vector3f point_2d = ep + (-z.dot(ep)) * z;
      const Eigen::Vector2f p2d(point_2d[0], point_2d[1]);
      if (cross2d((b2d - a2d), (p2d - a2d)) > 0 &&
          cross2d((c2d - b2d), (p2d - b2d)) > 0 &&
          cross2d((d2d - c2d), (p2d - c2d)) > 0 &&
          cross2d((a2d - d2d), (p2d - d2d)) > 0) {
        //set_indices.insert(index);
        ret->indices.push_back(index);
      }
    }

    //ret->indices = std::vector<int>(set_indices.begin(), set_indices.end());
    return ret;
  }
  
  pcl::PointIndices::Ptr
  FootstepState::cropPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                                ANNGrid::Ptr grid_search)
  {
    pcl::PointIndices::Ptr near_indices(new pcl::PointIndices);
    Eigen::Vector3f a = pose_ * Eigen::Vector3f(dimensions_[0]/2, dimensions_[1]/2, 0);
    Eigen::Vector3f b = pose_ * Eigen::Vector3f(-dimensions_[0]/2, dimensions_[1]/2, 0);
    Eigen::Vector3f c = pose_ * Eigen::Vector3f(-dimensions_[0]/2, -dimensions_[1]/2, 0);
    Eigen::Vector3f d = pose_ * Eigen::Vector3f(dimensions_[0]/2, -dimensions_[1]/2, 0);
    grid_search->approximateSearchInBox(a, b, c, d, *near_indices);
    return cropPointCloudExact(cloud, near_indices);
  }
  
  pcl::PointIndices::Ptr
  FootstepState::cropPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                                pcl::search::Octree<pcl::PointNormal>& tree)
  {
    pcl::PointNormal center;
    center.getVector3fMap() = Eigen::Vector3f(pose_.translation());
    center.z = 0.0;
    float r = 0.2;
    pcl::PointIndices::Ptr near_indices(new pcl::PointIndices);
    std::vector<float> distances;
    tree.radiusSearch(center, r, near_indices->indices, distances);
    return cropPointCloudExact(cloud, near_indices);
  }

  bool FootstepState::crossCheck(FootstepState::Ptr other)
  {
    Eigen::Vector3f a0, a1, a2, a3;
    Eigen::Vector3f b0, b1, b2, b3;
    vertices(a0, a1, a2, a3);
    other->vertices(b0, b1, b2, b3);
    Line2D a_01(a0, a1), a_12(a1, a2), a_23(a2, a3), a_30(a3, a0);
    Line2D b_01(b0, b1), b_12(b1, b2), b_23(b2, b3), b_30(b3, b0);
    return !(a_01.isCrossing(b_01) ||
             a_01.isCrossing(b_12) ||
             a_01.isCrossing(b_23) ||
             a_01.isCrossing(b_30) ||
             a_12.isCrossing(b_01) ||
             a_12.isCrossing(b_12) ||
             a_12.isCrossing(b_23) ||
             a_12.isCrossing(b_30) ||
             a_23.isCrossing(b_01) ||
             a_23.isCrossing(b_12) ||
             a_23.isCrossing(b_23) ||
             a_23.isCrossing(b_30) ||
             a_30.isCrossing(b_01) ||
             a_30.isCrossing(b_12) ||
             a_30.isCrossing(b_23) ||
             a_30.isCrossing(b_30));
  }
  
  FootstepState::Ptr
  FootstepState::projectToCloud(pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                                pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                                ANNGrid::Ptr grid_search,
                                pcl::search::Octree<pcl::PointNormal>& tree_2d,
                                pcl::PointCloud<pcl::PointNormal>::Ptr cloud_2d,
                                const Eigen::Vector3f& z,
                                unsigned int& error_state,
                                double outlier_threshold,
                                int max_iterations,
                                int min_inliers,
                                int foot_x_sampling_num,
                                int foot_y_sampling_num,
                                double vertex_threshold,
                                const bool skip_cropping)
  {
    // TODO: z is ignored
    // extract candidate points
    //pcl::PointIndices::Ptr indices = cropPointCloud(cloud_2d, tree_2d);
    // Before computing, check is it supported or not to omit recognition
    pcl::PointIndices::Ptr indices;
    FootstepSupportState presupport_state;
    if (skip_cropping) {
      presupport_state = isSupportedByPointCloudWithoutCropping(
        pose_, cloud, tree,
        indices, foot_x_sampling_num, foot_y_sampling_num, vertex_threshold);
    }
    indices = cropPointCloud(cloud, grid_search);
    if (indices->indices.size() < min_inliers) {
      error_state = projection_state::no_enough_inliers;
      return FootstepState::Ptr();
    }
    if (!skip_cropping) {
      presupport_state = isSupportedByPointCloud(
        pose_, cloud, tree,
        indices, foot_x_sampling_num, foot_y_sampling_num, vertex_threshold);
    }
    if (presupport_state == projection_state::success) {
      return FootstepState::Ptr(new FootstepState(leg_, pose_, dimensions_,
                                                  resolution_,
                                                  index_x_,
                                                  index_y_,
                                                  index_yaw_));
    }
    // estimate plane with ransac
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointNormal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setRadiusLimits(0.01, std::numeric_limits<double>::max ());
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(outlier_threshold);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setInputCloud(cloud);
    seg.setIndices(indices);
    seg.setMaxIterations(max_iterations);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      error_state = projection_state::no_plane;
      return FootstepState::Ptr();
    }
    else if (inliers->indices.size() < min_inliers) {
      error_state = projection_state::no_enough_inliers;
      return FootstepState::Ptr();
    }
    else {
      jsk_recognition_utils::Plane plane(coefficients->values);
      if (!plane.isSameDirection(z)) {
        plane = plane.flip();
      }
      Eigen::Vector3f n = plane.getNormal();
      Eigen::Vector3f x = pose_.matrix().block<3, 3>(0, 0) * Eigen::Vector3f::UnitX();
      if (acos(n.dot(x)) == 0) {
        error_state = projection_state::vertical_footstep;
        return FootstepState::Ptr();
      }
      Eigen::Vector3f rotation_axis = n.cross(x).normalized();
      Eigen::Vector3f new_x = Eigen::AngleAxisf(M_PI / 2.0, rotation_axis) * n;
      if (acos(new_x.dot(x)) > M_PI / 2.0) {
        new_x = - new_x;
      }
      Eigen::Vector3f new_y = n.cross(new_x);
      Eigen::Matrix3f new_rot_mat;
      new_rot_mat << new_x, new_y, n;
      Eigen::Quaternionf new_rot(new_rot_mat);
      Eigen::Vector3f p(pose_.translation());
      double alpha = (- plane.getD() - n.dot(p)) / (n.dot(z));
      Eigen::Vector3f q = p + alpha * z;
      
      Eigen::Affine3f new_pose = Eigen::Translation3f(q) * new_rot;
      // std::cout << "new_pose: " << std::endl << new_pose.matrix() << std::endl;
      // std::cout << "q: " << std::endl << q << std::endl;
      // std::cout << "new_rot_mat: " << std::endl << new_rot_mat << std::endl;
      //Eigen::Affine3f new_pose = new_rot * Eigen::Translation3f(q);
      // check is it enough points to support the footstep
      FootstepSupportState support_state;
      if (skip_cropping) {
        support_state = isSupportedByPointCloudWithoutCropping(
          new_pose, cloud, tree,
          inliers, foot_x_sampling_num, foot_y_sampling_num, vertex_threshold);
      }
      else {
        support_state = isSupportedByPointCloud(
          new_pose, cloud, tree,
          inliers, foot_x_sampling_num, foot_y_sampling_num, vertex_threshold);
      }
      if (support_state == NOT_SUPPORTED) {
        error_state = projection_state::no_enough_support;
        return FootstepState::Ptr();
      }
      else if (support_state == CLOSE_TO_SUPPORTED) {
        error_state = projection_state::close_to_success;
        return FootstepState::Ptr();
      }
      else {
        error_state = projection_state::success;
        return FootstepState::Ptr(new FootstepState(leg_, new_pose, dimensions_,
                                                    resolution_,
                                                    index_x_,
                                                    index_y_,
                                                    index_yaw_));
      }
    }
  }
  
  FootstepSupportState
  FootstepState::isSupportedByPointCloud(const Eigen::Affine3f& pose,
                                         pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                                         pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                                         pcl::PointIndices::Ptr inliers,
                                         const int foot_x_sampling_num,
                                         const int foot_y_sampling_num,
                                         const double vertex_threshold)
  {
    const double dx = dimensions_[0] / foot_x_sampling_num;
    const double dy = dimensions_[1] / foot_y_sampling_num;
    // vertices
    const Eigen::Vector3f ux = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f uy = Eigen::Vector3f::UnitY();
    const Eigen::Affine3f new_origin = pose *
      Eigen::Translation3f(- ux * dimensions_[0] / 2.0) *
      Eigen::Translation3f(- uy * dimensions_[1] / 2.0);
    const Eigen::Affine3f inv_pose = new_origin.inverse();
    
    bool occupiedp[foot_x_sampling_num][foot_y_sampling_num];
    // Initialize by false
    for (size_t i = 0; i < foot_x_sampling_num; i++) {
      for (size_t j = 0; j < foot_y_sampling_num; j++) {
        occupiedp[i][j] = false;
      }
    }
    
    for (size_t i = 0; i < inliers->indices.size(); i++) {
      pcl::PointNormal pp = cloud->points[inliers->indices[i]];
      const Eigen::Vector3f p = pp.getVector3fMap();
      const Eigen::Vector3f local_p = inv_pose * p;
      const int nx = floor(local_p[0] / dx);
      const int ny = floor(local_p[1] / dy);
      //std::cout << "abs_local_p2: " << std::abs(local_p[2]) << "/" << vertex_threshold << std::endl;
      if (0 <= nx && nx < foot_x_sampling_num &&
          0 <= ny && ny < foot_y_sampling_num &&
          std::abs(local_p[2]) < vertex_threshold) {
        occupiedp[nx][ny] = true;
      }
    }
    for (size_t i = 0; i < foot_x_sampling_num; i++) {
      for (size_t j = 0; j < foot_y_sampling_num; j++) {
        if (!occupiedp[i][j]) {
          return NOT_SUPPORTED;
        }
      }
    }

    Eigen::Vector3f a((pose * Eigen::Translation3f(ux * dimensions_[0] / 2 + uy * dimensions_[1] / 2)).translation());
    Eigen::Vector3f b((pose * Eigen::Translation3f(- ux * dimensions_[0] / 2 + uy * dimensions_[1] / 2)).translation());
    Eigen::Vector3f c((pose * Eigen::Translation3f(- ux * dimensions_[0] / 2 - uy * dimensions_[1] / 2)).translation());
    Eigen::Vector3f d((pose * Eigen::Translation3f(ux * dimensions_[0] / 2 - uy * dimensions_[1] / 2)).translation());
    pcl::PointNormal pa, pb, pc, pd, p;
    pa.getVector3fMap() = a;
    pb.getVector3fMap() = b;
    pc.getVector3fMap() = c;
    pd.getVector3fMap() = d;
    p.getVector3fMap() = Eigen::Vector3f(pose.translation());
    std::vector<int> kdl_indices;
    std::vector<float> kdl_distances;
    if (tree.radiusSearch(p, vertex_threshold, kdl_indices, kdl_distances, 1) > 0 &&
        tree.radiusSearch(pa, vertex_threshold, kdl_indices, kdl_distances, 1) > 0 &&
        tree.radiusSearch(pb, vertex_threshold, kdl_indices, kdl_distances, 1) > 0 &&
        tree.radiusSearch(pc, vertex_threshold, kdl_indices, kdl_distances, 1) > 0 &&
        tree.radiusSearch(pd, vertex_threshold, kdl_indices, kdl_distances, 1) > 0) {
      return SUPPORTED;
    }
    else {
      return CLOSE_TO_SUPPORTED;
    }
  }
  
  FootstepSupportState
  FootstepState::isSupportedByPointCloudWithoutCropping(
    const Eigen::Affine3f& pose,
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
    pcl::KdTreeFLANN<pcl::PointNormal>& tree,
    pcl::PointIndices::Ptr inliers,
    const int foot_x_sampling_num,
    const int foot_y_sampling_num,
    const double vertex_threshold)
  {
    const Eigen::Vector3f ux = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f uy = Eigen::Vector3f::UnitY();
    Eigen::Vector3f a((pose * Eigen::Translation3f(ux * dimensions_[0] / 2 + uy * dimensions_[1] / 2)).translation());
    Eigen::Vector3f b((pose * Eigen::Translation3f(- ux * dimensions_[0] / 2 + uy * dimensions_[1] / 2)).translation());
    Eigen::Vector3f c((pose * Eigen::Translation3f(- ux * dimensions_[0] / 2 - uy * dimensions_[1] / 2)).translation());
    Eigen::Vector3f d((pose * Eigen::Translation3f(ux * dimensions_[0] / 2 - uy * dimensions_[1] / 2)).translation());
    pcl::PointNormal pa, pb, pc, pd, p;
    pa.getVector3fMap() = a;
    pb.getVector3fMap() = b;
    pc.getVector3fMap() = c;
    pd.getVector3fMap() = d;
    p.getVector3fMap() = Eigen::Vector3f(pose.translation());
    std::vector<int> kdl_indices;
    std::vector<float> kdl_distances;
    size_t success_count = 0;
    
    if (tree.radiusSearch(p, vertex_threshold, kdl_indices, kdl_distances, 1) > 0) {
      ++success_count;
    }
    if (tree.radiusSearch(pa, vertex_threshold, kdl_indices, kdl_distances, 1) > 0) {
      ++success_count;
    }
    if (tree.radiusSearch(pb, vertex_threshold, kdl_indices, kdl_distances, 1) > 0) {
      ++success_count;
    }
    if (tree.radiusSearch(pc, vertex_threshold, kdl_indices, kdl_distances, 1) > 0) {
      ++success_count;
    }
    if (tree.radiusSearch(pd, vertex_threshold, kdl_indices, kdl_distances, 1) > 0) {
      ++success_count;
    }
    if (success_count == 5) {
      return SUPPORTED;
    }
    // else if (success_count > 0) {
    //   return CLOSE_TO_SUPPORTED;
    // }
    else {
      return NOT_SUPPORTED;
    }
  }
  

  
  FootstepState::Ptr FootstepState::fromROSMsg(const jsk_footstep_msgs::Footstep& f,
                                               const Eigen::Vector3f& size,
                                               const Eigen::Vector3f& resolution)
  {
    Eigen::Affine3f pose;
    tf::poseMsgToEigen(f.pose, pose);
    return FootstepState::Ptr(new FootstepState(
                                f.leg, pose,
                                size, resolution));
  }

}
