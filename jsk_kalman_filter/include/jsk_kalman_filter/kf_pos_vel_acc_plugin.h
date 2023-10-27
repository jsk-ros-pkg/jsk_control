// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#ifndef KF_POS_VEL_ACC_PLUGIN_H
#define KF_POS_VEL_ACC_PLUGIN_H

/* base class */
#include <jsk_kalman_filter/kf_base_plugin.h>

/* plugin */
#include <pluginlib/class_list_macros.h>

namespace kf_plugin
{
  enum CORRECT_MODE{POS = 0, VEL = 1, POS_VEL = 2,};

  class KalmanFilterPosVelAcc : public kf_plugin::KalmanFilter
  {
  public:

    /*
      state_dim_ = 2: p, v ; 3: p, v, b_a
      input_dim_ = 1: a; 2: a, d_b_a
      measure_dim_ = 1: p/v; 2: p + v
    */

    KalmanFilterPosVelAcc(): KalmanFilter(), estimate_acc_bias_(false) {}

    ~KalmanFilterPosVelAcc() {}

    void initialize(string name, int id);

    /* speical overwrite */
    bool prediction(const VectorXd input, const double timestamp, const vector<double> params = vector<double>(0));
    void setPredictionNoiseCovariance(const VectorXd& input_sigma_v);

    /* be sure that the first parma should be timestamp */
    void getPredictModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& state_transition_model, MatrixXd& control_input_model) const override;
    void getCorrectModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& observation_model) const override;

    void setInputSigma( VectorXd input_sigma_v);
  private:
    bool estimate_acc_bias_;

  };
};

#endif
