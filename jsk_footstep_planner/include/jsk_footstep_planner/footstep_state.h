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

namespace jsk_footstep_planner
{

  namespace projection_state
  {
    const unsigned int success = 1;
    const unsigned int no_pointcloud = 2;
    const unsigned int no_enough_support = 4;
    const unsigned int no_plane = 8;
    const unsigned int no_enough_inliers = 16;
  }
  
  class FootstepState
  {
  public:
    typedef boost::shared_ptr<FootstepState> Ptr;
    FootstepState(int leg,
                  const Eigen::Affine3f& pose,
                  const Eigen::Vector3f& dimensions):
      leg_(leg), pose_(pose), dimensions_(dimensions)
      {}
    virtual jsk_footstep_msgs::Footstep::Ptr toROSMsg();
    virtual FootstepState::Ptr
    projectToCloud(pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                   pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                   const Eigen::Vector3f& z,
                   unsigned int& error_state,
                   double outlier_threshold,
                   int max_iterations,
                   int min_inliers);
    pcl::PointIndices::Ptr
    cropPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
        
    virtual Eigen::Affine3f getPose() { return pose_; }
    virtual void setPose(const Eigen::Affine3f& pose) { pose_ = pose; }
  protected:
    Eigen::Affine3f pose_;
    const Eigen::Vector3f dimensions_;
    const int leg_;
    int hash_x_;
    int hash_y_;
    int hash_yaw_;
  private:
    
  };
}

#endif
