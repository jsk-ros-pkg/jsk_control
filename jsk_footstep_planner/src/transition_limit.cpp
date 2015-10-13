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

#include "jsk_footstep_planner/transition_limit.h"
#include <pcl/common/eigen.h>

namespace jsk_footstep_planner
{
  TransitionLimitRP::TransitionLimitRP(
    double roll_max,
    double pitch_max):
    roll_max_(roll_max), pitch_max_(pitch_max)
  {
  }

  bool TransitionLimitRP::check(FootstepState::Ptr from,
                                FootstepState::Ptr to) const
  {
    const Eigen::Affine3f diff = to->getPose() * from->getPose().inverse();
    float roll, pitch, yaw;
    pcl::getEulerAngles(diff, roll, pitch, yaw);
    return (std::abs(roll) < roll_max_ &&
            std::abs(pitch) < pitch_max_);
  }
  
  TransitionLimitXYZRPY::TransitionLimitXYZRPY(
    double x_max,
    double y_max,
    double z_max,
    double roll_max,
    double pitch_max,
    double yaw_max):
    x_max_(x_max), y_max_(y_max), z_max_(z_max),
    roll_max_(roll_max), pitch_max_(pitch_max), yaw_max_(yaw_max)
  {
  }
  
  bool TransitionLimitXYZRPY::check(FootstepState::Ptr from,
                                    FootstepState::Ptr to) const
  {
    // from * trans = to
    const Eigen::Affine3f diff = to->getPose() * from->getPose().inverse();
    const Eigen::Vector3f diff_pos(diff.translation());
    if (std::abs(diff_pos[0]) < x_max_ &&
        std::abs(diff_pos[1]) < y_max_ &&
        std::abs(diff_pos[2]) < z_max_) {
      float roll, pitch, yaw;
      pcl::getEulerAngles(diff, roll, pitch, yaw);
      return (std::abs(roll) < roll_max_ &&
              std::abs(pitch) < pitch_max_ &&
              std::abs(yaw) < yaw_max_);
    }
    else {
      return false;
    }
  }
}
