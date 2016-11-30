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


#ifndef JSK_FOOTSTEP_PLANNER_FOOTSTEP_STATE_H_
#define JSK_FOOTSTEP_PLANNER_FOOTSTEP_STATE_H_

#include <jsk_footstep_msgs/Footstep.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/octree.h>

#include "jsk_footstep_planner/ann_grid.h"
#include "jsk_footstep_planner/util.h"
#include "jsk_footstep_planner/footstep_parameters.h"

namespace jsk_footstep_planner
{

  namespace projection_state
  {
    const unsigned int success = 1;
    const unsigned int no_pointcloud = 2;
    const unsigned int no_enough_support = 4;
    const unsigned int no_plane = 8;
    const unsigned int no_enough_inliers = 16;
    const unsigned int close_to_success = 32;
    const unsigned int transition_limit = 64;
    const unsigned int vertical_footstep = 128;
    const unsigned int no_enough_inliers_ratio = 256;
  }

  std::string projectStateToString(unsigned int state);
  
  enum FootstepSupportState
  {
    NOT_SUPPORTED,
    SUPPORTED,
    CLOSE_TO_SUPPORTED,
  };
  
  class FootstepState
  {
  public:
    typedef boost::shared_ptr<FootstepState> Ptr;
    FootstepState(int leg,
                  const Eigen::Affine3f& pose,
                  const Eigen::Vector3f& dimensions,
                  const Eigen::Vector3f& resolution):
      leg_(leg), pose_(pose), dimensions_(dimensions), resolution_(resolution)
    {
      debug_print_ = false;
      float x = pose_.translation()[0];
      float y = pose_.translation()[1];
      float roll, pitch, yaw;
      pcl::getEulerAngles(pose_, roll, pitch, yaw);
      index_x_ = x / resolution_[0];
      index_y_ = y / resolution_[1];
      index_yaw_ = yaw / resolution_[2];
    }

    FootstepState(int leg,
                  const Eigen::Affine3f& pose,
                  const Eigen::Vector3f& dimensions,
                  const Eigen::Vector3f& resolution,
                  int index_x,
                  int index_y,
                  int index_yaw):
      leg_(leg), pose_(pose), dimensions_(dimensions), resolution_(resolution),
      index_x_(index_x), index_y_(index_y), index_yaw_(index_yaw)
    {
      debug_print_ = false;
    }

    static
    FootstepState::Ptr fromROSMsg(const jsk_footstep_msgs::Footstep& f,
                                  const Eigen::Vector3f& size,
                                  const Eigen::Vector3f& resolution);
    
    inline float cross2d(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const
    {
      return a[0] * b[1] - a[1] * b[0];
    }
    virtual jsk_footstep_msgs::Footstep::Ptr toROSMsg();
    virtual jsk_footstep_msgs::Footstep::Ptr toROSMsg(const Eigen::Vector3f& ioffset);
#if 0
    virtual FootstepState::Ptr
    projectToCloud(pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                   pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                   ANNGrid::Ptr grid_search,
                   pcl::search::Octree<pcl::PointNormal>& tree_2d,
                   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_2d,
                   const Eigen::Vector3f& z,
                   unsigned int& error_state,
                   double outlier_threshold,
                   int max_iterations,
                   int min_inliers,
                   int foot_x_sampling_num = 3,
                   int foot_y_sampling_num = 3,
                   double vertex_threshold = 0.02,
                   const bool skip_cropping = true,
                   const bool use_normal = false,
                   double normal_distance_weight = 0.2,
                   double normal_opening_angle = 0.2,
                   double min_ratio_of_inliers = 0.8);
#endif
    virtual FootstepState::Ptr
    projectToCloud(pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                   pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                   ANNGrid::Ptr grid_search,
                   pcl::search::Octree<pcl::PointNormal>& tree_2d,
                   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_2d,
                   const Eigen::Vector3f& z,
                   unsigned int& error_state,
                   FootstepParameters &parameters);
    
#if 0
    pcl::PointIndices::Ptr
    cropPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                   pcl::search::Octree<pcl::PointNormal>& tree);
#endif
    pcl::PointIndices::Ptr
    cropPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                   ANNGrid::Ptr grid_search,
                   double padding_x = 0.0, double padding_y = 0.0);

    pcl::PointIndices::Ptr
    cropPointCloudExact(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                        pcl::PointIndices::Ptr near_indices,
                        double padding_x = 0.0, double padding_y = 0.0);

    template <class PointT>
    PointT toPoint()
    {
      PointT p;
      p.getVector3fMap() = Eigen::Vector3f(pose_.translation());
      return p;
    }

    inline void vertices(Eigen::Vector3f& a,
                         Eigen::Vector3f& b,
                         Eigen::Vector3f& c,
                         Eigen::Vector3f& d,
                         double collision_padding = 0)
    {
      const Eigen::Vector3f ux = Eigen::Vector3f::UnitX();
      const Eigen::Vector3f uy = Eigen::Vector3f::UnitY();
      double dim0 = dimensions_[0] + collision_padding;
      double dim1 = dimensions_[1] + collision_padding;
      a = Eigen::Vector3f((pose_ * Eigen::Translation3f(  ux * dim0 / 2 + uy * dim1 / 2)).translation());
      b = Eigen::Vector3f((pose_ * Eigen::Translation3f(- ux * dim0 / 2 + uy * dim1 / 2)).translation());
      c = Eigen::Vector3f((pose_ * Eigen::Translation3f(- ux * dim0 / 2 - uy * dim1 / 2)).translation());
      d = Eigen::Vector3f((pose_ * Eigen::Translation3f(  ux * dim0 / 2 - uy * dim1 / 2)).translation());
    }
    
    /**
     * @brief
     * return true if this and other are collision free.
     */
    virtual bool crossCheck(FootstepState::Ptr other, float collision_padding = 0);
    
    virtual Eigen::Affine3f getPose() const { return pose_; }
    virtual void setPose(const Eigen::Affine3f& pose)
    {
      pose_ = pose;
    }
    
    virtual int getLeg() const { return leg_; }
    virtual Eigen::Vector3f getDimensions() const { return dimensions_; }
    bool operator==(FootstepState& other)
    {
      return ((index_x_ == other.index_x_) &&
              (index_y_ == other.index_y_) &&
              (index_yaw_ == other.index_yaw_));
    }

    virtual Eigen::Vector3f getResolution() const { return resolution_; }
      

    inline virtual int indexX() { return index_x_; }
    inline virtual int indexY() { return index_y_; }
    inline virtual int indexT() { return index_yaw_; }


    virtual FootstepSupportState
    isSupportedByPointCloud(const Eigen::Affine3f& pose,
                            pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                            pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                            pcl::PointIndices::Ptr inliers,
                            const int foot_x_sampling_num,
                            const int foot_y_sampling_num,
                            const double vertex_threshold);
    virtual FootstepSupportState
    isSupportedByPointCloudWithoutCropping(const Eigen::Affine3f& pose,
                                           pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                                           pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                                           pcl::PointIndices::Ptr inliers,
                                           const int foot_x_sampling_num,
                                           const int foot_y_sampling_num,
                                           const double vertex_threshold);
    
    virtual Eigen::Affine3f midcoords(const FootstepState& other);
  protected:
    Eigen::Affine3f pose_;
    const Eigen::Vector3f dimensions_;
    const Eigen::Vector3f resolution_; // not memory efficient?
    const int leg_;
    int index_x_;
    int index_y_;
    int index_yaw_;
    bool debug_print_;
  private:
    
  };

  inline size_t hash_value(const FootstepState::Ptr& s)
  {
    return std::abs(s->indexX()) << (25 + 14) + std::abs(s->indexY()) << 14
      + std::abs(s->indexT());
  }
  
}

#endif
