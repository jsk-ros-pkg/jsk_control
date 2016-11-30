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


#ifndef JSK_FOOTSTEP_PARAMETERS_H_
#define JSK_FOOTSTEP_PARAMETERS_H_

namespace jsk_footstep_planner
{
  struct FootstepParameters
  {
    FootstepParameters ():
      use_transition_limit(false), use_global_transition_limit(false),
      plane_estimation_use_normal(false), skip_cropping(false),
      local_move_x_num(3), local_move_y_num(3), local_move_theta_num(3),
      plane_estimation_max_iterations(100), plane_estimation_min_inliers(100),
      support_check_x_sampling(3), support_check_y_sampling(3),
      local_move_x(0.1), local_move_y(0.05), local_move_theta(0.1),
      local_move_x_offset(0.1), local_move_y_offset(0.05), local_move_theta_offset(0.0),
      transition_limit_x(0.3), transition_limit_y(0.5), transition_limit_z(0.4),
      transition_limit_roll(0.3), transition_limit_pitch(0.3), transition_limit_yaw(0.3),
      global_transition_limit_roll(0.3), global_transition_limit_pitch(0.3),
      obstacle_resolution(0.1), goal_pos_thr(0.1), goal_rot_thr(0.17),
      plane_estimation_normal_distance_weight(0.2), plane_estimation_normal_opening_angle(0.2),
      plane_estimation_min_ratio_of_inliers(0.8), plane_estimation_outlier_threshold(0.02),
      support_check_vertex_neighbor_threshold(0.02),
      support_padding_x(0.0), support_padding_y(0.0)
    {
    };
    bool use_transition_limit;
    bool use_global_transition_limit;
    bool plane_estimation_use_normal;
    bool skip_cropping;
    int local_move_x_num;
    int local_move_y_num;
    int local_move_theta_num;
    int plane_estimation_max_iterations;
    int plane_estimation_min_inliers;
    int support_check_x_sampling;
    int support_check_y_sampling;
    double local_move_x;
    double local_move_y;
    double local_move_theta;
    double local_move_x_offset;
    double local_move_y_offset;
    double local_move_theta_offset;
    double transition_limit_x;
    double transition_limit_y;
    double transition_limit_z;
    double transition_limit_roll;
    double transition_limit_pitch;
    double transition_limit_yaw;
    double global_transition_limit_roll;
    double global_transition_limit_pitch;
    double obstacle_resolution;
    double goal_pos_thr;
    double goal_rot_thr;
    double plane_estimation_normal_distance_weight;
    double plane_estimation_normal_opening_angle;
    double plane_estimation_min_ratio_of_inliers;
    double plane_estimation_outlier_threshold;
    double support_check_vertex_neighbor_threshold;
    double support_padding_x;
    double support_padding_y;
    double collision_padding;
  };
}
#endif
