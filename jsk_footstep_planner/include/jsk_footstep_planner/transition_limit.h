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


#ifndef JSK_FOOTSTEP_PLANNER_TRANSITION_LIMIT_H_
#define JSK_FOOTSTEP_PLANNER_TRANSITION_LIMIT_H_

#include "jsk_footstep_planner/footstep_state.h"

namespace jsk_footstep_planner
{

  /**
   * @brief
   * virtual class to provide limit of transition of footstep.
   */
  class TransitionLimit
  {
  public:
    typedef boost::shared_ptr<TransitionLimit> Ptr;
    virtual bool check(FootstepState::Ptr from,
                       FootstepState::Ptr to) const = 0;
                       
  protected:
  private:
    
  };
  
  /**
   * @brief
   * Class to provide limit of transition of footstep about Roll and Pitch.
   * This class is designed for global soundness of footstep.
   */
  class TransitionLimitRP: public TransitionLimit
  {
  public:
    typedef boost::shared_ptr<TransitionLimitRP> Ptr;
    TransitionLimitRP(double roll_max,
                      double pitch_max);
    virtual bool check(FootstepState::Ptr from,
                       FootstepState::Ptr to) const;
  protected:
    const double roll_max_;
    const double pitch_max_;

  private:
  };

  /**
   * @brief
   * class to provide limit of transition of footstep with 6 Full parameters.
   */
  class TransitionLimitXYZRPY: public TransitionLimit
  {
  public:
    typedef boost::shared_ptr<TransitionLimitXYZRPY> Ptr;
    TransitionLimitXYZRPY(double x_max,
                          double y_max,
                          double z_max,
                          double roll_max,
                          double pitch_max,
                          double yaw_max);
    virtual bool check(FootstepState::Ptr from,
                       FootstepState::Ptr to) const;
  protected:
    const double x_max_;
    const double y_max_;
    const double z_max_;
    const double roll_max_;
    const double pitch_max_;
    const double yaw_max_;
  private:
    
  };
}

#endif
