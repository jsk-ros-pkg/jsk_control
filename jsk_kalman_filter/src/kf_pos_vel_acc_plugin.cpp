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

/* header */
#include <jsk_kalman_filter/kf_pos_vel_acc_plugin.h>

namespace kf_plugin
{

  void KalmanFilterPosVelAcc::initialize(string name, int id)
  {
    state_dim_ = 2; // default mode: no acc bias estimation

    state_names_ = {"pos", "vel", "bias"};
    input_names_ = {"acc", "d_bias"};
    measure_names_ = {"pos", "vel"};

    KalmanFilter::initialize(name, id);

  }

  /* Caution: this API is called in the init phase, that is, once */
  void KalmanFilterPosVelAcc::setPredictionNoiseCovariance(const VectorXd& input_sigma_v)
  {
    VectorXd input_sigma_v_temp = input_sigma_v;
    if(input_sigma_v.size() == 2) // contain the acc_bias sigma
      {
        if(input_sigma_v(1) == 0)
          {// no bias estimation because the sigma is zero

            double acc_sigma = input_sigma_v(0);
            input_sigma_v_temp.resize(1);
            input_sigma_v_temp(0) = acc_sigma;

            std::cout << name_ << ": pos_vel_acc mode" << std::endl;
          }
        else
          {
            estimate_acc_bias_ = true;

            /* re-init the dim and relavant var */
            VectorXd prev_init_state = init_state_;
            assert(prev_init_state.size() == 2);
            state_dim_ = 3;
            resetState();
            init_state_(0) = prev_init_state(0);
            init_state_(1) = prev_init_state(1);
            std::cout << name_ << ": pos_vel_acc_bias mode" << std::endl;
          }
      }

    KalmanFilter::setPredictionNoiseCovariance(input_sigma_v_temp);
  }

  bool KalmanFilterPosVelAcc::prediction(const VectorXd input,
                                         const double timestamp,
                                         const vector<double> params)
  {
    bool ret = true;
    if(input.size() == 1 && estimate_acc_bias_) {
        // special process for bias estimation
        VectorXd input_temp = VectorXd::Zero(2);
        input_temp(0) = input(0);
        ret = KalmanFilter::prediction(input_temp, timestamp, params);
      }
    else {
        ret = KalmanFilter::prediction(input, timestamp, params);
    }
    return ret;
  }

  /* be sure that the first param should be timestamp */
  void KalmanFilterPosVelAcc::getPredictModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& state_transition_model, MatrixXd& control_input_model) const
  {
    assert(params.size() == 1);

    float dt = params[0];

    assert(dt >= 0);

    if(estimate_acc_bias_)
      {
        Matrix3d state_transition_model_temp;
        state_transition_model_temp << 1, dt, -dt*dt/2, 0, 1, -dt, 0, 0, 1;
        state_transition_model = state_transition_model_temp;

        Matrix<double, 3, 2> control_input_model_temp;
        control_input_model_temp << (dt * dt)/2, 0, dt, 0, 0, 1;
        control_input_model = control_input_model_temp;
      }
    else
      {
        Matrix2d state_transition_model_temp;
        state_transition_model_temp << 1, dt, 0, 1;
        state_transition_model = state_transition_model_temp;

        Matrix<double, 2, 1> control_input_model_temp;
        control_input_model_temp << (dt * dt)/2, dt;
        control_input_model = control_input_model_temp;
      }
  }

  /* be sure that the first parma is timestamp */
  void KalmanFilterPosVelAcc::getCorrectModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& observation_model) const
  {
    /* params: correct mode */
    assert(params.size() == 1);
    assert((int)params[0] <= POS_VEL);

    if(estimate_acc_bias_)
      {
        MatrixXd observation_model_temp(2,3);
        observation_model_temp << 1, 0, 0, 0, 1, 0;
        switch((int)params[0])
          {
          case POS:
            {
              //observation_model_temp << 1, 0, 0;
              observation_model = observation_model_temp.block(0, 0, 1, 3);
              break;
            }
          case VEL:
            {
              //observation_model_temp << 0, 1, 0;
              observation_model = observation_model_temp.block(1, 0, 1, 3);
              break;
            }
          case POS_VEL:
            {
              observation_model = observation_model_temp;
              break;
            }
          }
      }
    else
      {
        Matrix2d observation_model_temp = Matrix2d::Identity();

        switch((int)params[0])
          {
          case POS:
            {
              //observation_model_temp << 1, 0;
              observation_model = observation_model_temp.block(0, 0, 1, 2);
              break;
            }
          case VEL:
            {
              //observation_model_temp << 0, 1;
              observation_model = observation_model_temp.block(1, 0, 1, 2);
              break;
            }
          case POS_VEL:
            {
              observation_model = observation_model_temp;
              break;
            }
          }
      }
  }

};

PLUGINLIB_EXPORT_CLASS(kf_plugin::KalmanFilterPosVelAcc, kf_plugin::KalmanFilter);

