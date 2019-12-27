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


#ifndef JSK_FOOTSTEP_PLANNER_FOOTSTEP_STATE_DISCRETE_CLOSE_LIST_H_
#define JSK_FOOTSTEP_PLANNER_FOOTSTEP_STATE_DISCRETE_CLOSE_LIST_H_

#include "jsk_footstep_planner/footstep_state.h"
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>

namespace jsk_footstep_planner
{
  class FootstepStateDiscreteCloseListLocal
  {
    // x_offset, x_offset + 1, ...., x_offset + X - 1
  public:
    typedef boost::shared_ptr<FootstepStateDiscreteCloseListLocal> Ptr;
    FootstepStateDiscreteCloseListLocal(
      int x_offset, int y_offset, int theta_offset,
      size_t x_num, size_t y_num, size_t theta_num);
    
    inline FootstepState::Ptr get(int x, int y, int theta)
    {
      return data_[x - x_offset_][y - y_offset_][theta - theta_offset_];
    }

    inline void add(FootstepState::Ptr state)
    {
      int x = state->indexX();
      int y = state->indexY();
      int theta = state->indexT();
      if (!data_[x - x_offset_][y - y_offset_][theta - theta_offset_]) {
        size_++;
      }
      data_[x - x_offset_][y - y_offset_][theta - theta_offset_] = state;
    }

    // Use push_back to inser points
    template <class PointT>
    void insertToPointCloud(pcl::PointCloud<PointT>& output)
    {
      for (size_t xi = 0; xi < x_num_; xi++) {
        for (size_t yi = 0; yi < y_num_; yi++) {
          for (size_t thetai = 0; thetai < theta_num_; thetai++) {
            if (data_[xi][yi][thetai]) {
              FootstepState::Ptr state = data_[xi][yi][thetai];
              PointT p = state->toPoint<PointT>();
              output.points.push_back(p);
            }
          }
        }
      }
    }

    inline size_t size() { return size_; }
  protected:
    size_t size_;
    const size_t x_num_;
    const size_t y_num_;
    const size_t theta_num_;
    const int x_offset_;
    const int y_offset_;
    const int theta_offset_;
    std::vector<std::vector<std::vector<FootstepState::Ptr> > > data_;
  private:
    
  };

  /**
   * @brief
   *
   * FootstepStateDiscreteCloseList is a special clas
   * to use for close list of FootstepState
   */
  class FootstepStateDiscreteCloseList
  {
  public:
    typedef boost::shared_ptr<FootstepStateDiscreteCloseList> Ptr;
    typedef boost::tuple<int, int, int> VolumeKey;
    FootstepStateDiscreteCloseList(const size_t local_x_num,
                                   const size_t local_y_num,
                                   const size_t local_theta_num);
    inline int keyDivide(int x, int y)
    {
      if (x < 0) {
        // return -1, while -1 <= x <= -local_num
        // return -2, while -local_num -1 <= x <= - 2*local_num
        // ...
        return (x + 1)/ y - 1;
      }
      else {
        // return 0, while 0 <= x <= local_num -1
        // return 1, while local_num <= x <= 2*local_num -1
        // ...
        return x / y;
      }
    }
    inline VolumeKey volumeKey(
      int xi, int yi, int ti)
    {
      int kx = keyDivide(xi, local_x_num_);
      int ky = keyDivide(yi, local_y_num_);
      int kt = keyDivide(ti, local_theta_num_);
      return boost::make_tuple(kx, ky, kt);
    }

    inline size_t size()
    {
      std::map<VolumeKey, FootstepStateDiscreteCloseListLocal::Ptr>::iterator it;
      size_t s = 0;
      for (it = local_volumes_.begin(); it != local_volumes_.end(); ++it) {
        s += it->second->size();
      }
      return s;
    }
    
    inline void push_back(FootstepState::Ptr state)
    {
      int xi = state->indexX();
      int yi = state->indexY();
      int ti = state->indexT();
      VolumeKey local_volume_key = volumeKey(xi, yi, ti);
      std::map<VolumeKey, FootstepStateDiscreteCloseListLocal::Ptr>::iterator it
        = local_volumes_.find(local_volume_key);
      if (it != local_volumes_.end()) { // found!
        it->second->add(state);
      }
      else {
        // add new local volume
        FootstepStateDiscreteCloseListLocal::Ptr new_local(
          new FootstepStateDiscreteCloseListLocal(local_volume_key.get<0>() * local_x_num_,
                                                  local_volume_key.get<1>() * local_y_num_,
                                                  local_volume_key.get<2>() * local_theta_num_,
                                                  local_x_num_,
                                                  local_y_num_,
                                                  local_theta_num_));
        local_volumes_[local_volume_key] = new_local;
        new_local->add(state);
      }
    }

    
    inline bool find(FootstepState::Ptr state)
    {
      int xi = state->indexX();
      int yi = state->indexY();
      int ti = state->indexT();
      VolumeKey key = volumeKey(xi, yi, ti);
      std::map<VolumeKey, FootstepStateDiscreteCloseListLocal::Ptr>::iterator it
        = local_volumes_.find(key);
      if (it != local_volumes_.end()) { // found!
        return (bool)it->second->get(xi, yi, ti);
      }
      else {
        return false;
      }
    }

    // This method may be slow
    template <class PointT>
    void toPointCloud(pcl::PointCloud<PointT>& output)
    {
      output.points.reserve(size());
      for (std::map<VolumeKey, FootstepStateDiscreteCloseListLocal::Ptr>::iterator it
             = local_volumes_.begin();
           it != local_volumes_.end();
           ++it) {
        it->second->insertToPointCloud<PointT>(output);
      }
    }
    
  protected:
    const size_t local_x_num_;
    const size_t local_y_num_;
    const size_t local_theta_num_;
    // local volume := [int][int][int] -> FootstepState::Ptr
    // global volume := [int][int][int] -> local volume
    std::map<VolumeKey, FootstepStateDiscreteCloseListLocal::Ptr> local_volumes_;
  private:
    
  };
}

#endif
