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

#include "jsk_footstep_planner/line2d.h"

namespace jsk_footstep_planner
{
  Line2D::Line2D(const Eigen::Vector3f& p, const Eigen::Vector3f& q):
    u_(p[0], p[1]), v_(q[0], q[1])
  {
  }

  bool Line2D::isCrossing(Line2D& other)
  {
    const float d = ((v_[0] - u_[0]) * (other.v_[1] - other.u_[1])
                     - (v_[1] - u_[1]) * (other.v_[0] - other.u_[0]));
    if (d == 0) {               // parallel
      return false;
    }
    const float u = ((other.u_[0] - u_[0])*(other.v_[1] - other.u_[1])
                     - (other.u_[1] - u_[1])*(other.v_[0] - other.u_[0]))/d;
    const float v = ((other.u_[0] - u_[0])*(v_[1] - u_[1])
                     - (other.u_[1] - u_[1])*(v_[0] - u_[0]))/d;
    if (u < 0.0 || u > 1.0) {
      return false;
    }
    if (v < 0.0 || v > 1.0) {
      return false;
    }
    return true;
  }
}

