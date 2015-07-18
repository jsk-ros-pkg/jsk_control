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

#include "jsk_footstep_planner/footstep_state_discrete_close_list.h"

namespace jsk_footstep_planner
{
  FootstepStateDiscreteCloseListLocal::FootstepStateDiscreteCloseListLocal(
    int x_offset, int y_offset, int theta_offset,
    size_t x_num, size_t y_num, size_t theta_num):
    x_num_(x_num), y_num_(y_num), theta_num_(theta_num),
    x_offset_(x_offset), y_offset_(y_offset), theta_offset_(theta_offset),
    size_(0)
  {
    // initialize data_
    data_ = std::vector<std::vector<std::vector<FootstepState::Ptr> > >(x_num_);
    for (size_t xi = 0; xi < x_num_; xi++) {
      data_[xi] = std::vector<std::vector<FootstepState::Ptr> >(y_num_);
      for (size_t yi = 0; yi < y_num_; yi++) {
        data_[xi][yi] = std::vector<FootstepState::Ptr>(theta_num_);
      }
    }
  }

  FootstepStateDiscreteCloseList::FootstepStateDiscreteCloseList(
    const size_t local_x_num,
    const size_t local_y_num,
    const size_t local_theta_num):
    local_x_num_(local_x_num),
    local_y_num_(local_y_num),
    local_theta_num_(local_theta_num)
  {
  }
    
}
