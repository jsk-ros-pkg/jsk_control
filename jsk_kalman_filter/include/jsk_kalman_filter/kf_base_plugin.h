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

#ifndef KALMAN_FILTER_PLUGIN_H
#define KALMAN_FILTER_PLUGIN_H

/* math */
#include <Eigen/Core>
#include <Eigen/Dense>

/* util */
#include <iostream>
#include<iomanip>
#include <vector>
#include <deque>

/* for mutex */
#include <mutex>
#include <thread>

using namespace Eigen;
using namespace std;

namespace kf_plugin
{
  class CorrectHandler
  {
  public:
    CorrectHandler(){}
    ~CorrectHandler(){}

    /* assumption: the sigma is constant for whole system */
    std::vector<double> model_params_;
    Eigen::MatrixXd measurement_noise_covariance_;
    Eigen::VectorXd measure_value_;

    double timestamp_;
  };
  using CorrectHandlerPtr = std::shared_ptr<CorrectHandler>;

  class PredictHandler
  {
  public:
    PredictHandler():correct_handlers_(0)
    {
      next_handler_ = nullptr;
    }
    ~PredictHandler(){}

    std::vector<double> model_params_;
    Eigen::VectorXd input_;
    double timestamp_;

    VectorXd estimate_state_;
    MatrixXd estimate_covariance_;

    std::shared_ptr<PredictHandler> next_handler_;
    std::vector<CorrectHandlerPtr> correct_handlers_;
  };
  using PredictHandlerPtr = std::shared_ptr<PredictHandler>;

  class KalmanFilter
  {
  public:
    KalmanFilter():
      input_start_flag_(false), measure_start_flag_(false),debug_verbose_(false),
      state_dim_(0), predict_handlers_(0), buf_size_(10)
    {
    }

    virtual ~KalmanFilter(){}

    virtual void initialize(string name, int id)
    {
      name_ = name;
      id_ = id;
      if(state_dim_ == 0) throw std::runtime_error("the state dimension is zero");

      init_state_ = VectorXd::Zero(state_dim_);
    }

    virtual bool prediction(const VectorXd input, /* the vector of input */
                            const double timestamp, /* the timestamp of prediction state(which will be the timestamp for whole system) */
                            const vector<double> params = vector<double>(0) /* the vector of param for predict model */)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);

      if(!getFilteringFlag()) return false;

      /* update the model */
      VectorXd estimate_state = (predict_handlers_.size() > 0)?predict_handlers_.back()->estimate_state_:init_state_;
      MatrixXd state_transition_model, control_input_model;
      getPredictModel(params, estimate_state, state_transition_model, control_input_model);
      /* propagation of state */
      VectorXd propagate_state;
      statePropagation(estimate_state, input, state_transition_model, control_input_model,
                       propagate_state);
      /* create new predict handler and push back to predict_handlers vector */
      PredictHandlerPtr new_predict_handler = PredictHandlerPtr(new PredictHandler());
      new_predict_handler->model_params_ = params; // the predict model may change due to the repropagation
      new_predict_handler->input_ = input;
      new_predict_handler->estimate_state_ = propagate_state;
      new_predict_handler->estimate_covariance_ = MatrixXd::Zero(state_dim_, state_dim_); // unknown, set negative
      new_predict_handler->timestamp_ = timestamp;
      if(predict_handlers_.size() > 0) predict_handlers_.back()->next_handler_ = new_predict_handler;
      predict_handlers_.push_back(new_predict_handler);
      /* propagation of covariance */
      if(last_estimated_cov_handler_ == nullptr)
        {
          last_estimated_cov_handler_ = predict_handlers_.back();
          last_estimated_cov_handler_->estimate_covariance_ = control_input_model * input_noise_covariance_ * control_input_model.transpose();
        }
      else
        {
          MatrixXd prev_covariance = last_estimated_cov_handler_->estimate_covariance_;
          last_estimated_cov_handler_ = last_estimated_cov_handler_->next_handler_; //incremental
          assert(last_estimated_cov_handler_ != nullptr);
          getPredictModel(last_estimated_cov_handler_->model_params_,
                          last_estimated_cov_handler_->estimate_state_,
                          state_transition_model, control_input_model);
          covariancePropagation(prev_covariance,
                                state_transition_model, control_input_model,
                                input_noise_covariance_,
                                last_estimated_cov_handler_->estimate_covariance_);
        }

      /* remove the oldest predict handler */
      if(predict_handlers_.size() > buf_size_) predict_handlers_.pop_front();

      if(debug_verbose_)
        {
          cout << name_ << "prediction: " << endl;
          cout << "state transition model: \n" << state_transition_model << endl;
          cout << "control input model: \n" << control_input_model << endl;

          cout << "state: \n" << propagate_state.transpose() << endl;
          cout << "covariance: \n" << last_estimated_cov_handler_->estimate_covariance_ << endl;
        }

      return true;
    }

    virtual bool correction(const VectorXd measurement, /* the vector of measurement */
                            const VectorXd noise_sigma, /* the vector of measure sigma */
                            const double timestamp = -1, /* timestamp of the measure state */
                            const vector<double> params = vector<double>(0), /* the vector of param for correct model, the first param should be timestamp */
                            const double outlier_thresh = 0 /*  check the outlier */)
    {
      /* lock the whole process, since several sensor may acess in the same time */
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);

      if(!getFilteringFlag()) return false;

      /* get time sync predict_handler */
      PredictHandlerPtr sync_predict_handler = getTimeSyncPropagationResult(timestamp);
      if(sync_predict_handler == nullptr) return false;

      /* update the model */
      MatrixXd observation_model, measurement_noise_covariance;
      getCorrectModel(params, sync_predict_handler->estimate_state_, observation_model);
      getMeasurementNoiseCovariance(noise_sigma, measurement_noise_covariance);

      /* correction */
      VectorXd old_estimate_state = sync_predict_handler->estimate_state_;
      MatrixXd old_estimate_covariance = sync_predict_handler->estimate_covariance_;
      if(!correctionCore(old_estimate_state, old_estimate_covariance,
                        measurement, observation_model,
                        measurement_noise_covariance,
                        outlier_thresh,
                        sync_predict_handler->estimate_state_,
                        sync_predict_handler->estimate_covariance_))
        return false; // outlier

      /* add the correct handler to the predict handler */
      CorrectHandlerPtr new_correct_handler = CorrectHandlerPtr(new CorrectHandler());
      new_correct_handler->model_params_ = params;
      new_correct_handler->measurement_noise_covariance_ = measurement_noise_covariance;
      new_correct_handler->measure_value_ = measurement;
      new_correct_handler->timestamp_ = (timestamp==-1)?sync_predict_handler->timestamp_:timestamp;
      sync_predict_handler->correct_handlers_.push_back(new_correct_handler);

      /* repropagate state and covariance */
      rePropagation(sync_predict_handler);

      return true;
    }

    const bool measurementOutlierCheck(const VectorXd& measurement, const VectorXd& estimate_state, const MatrixXd& estimate_covariance, const MatrixXd& observation_model, const MatrixXd& measurement_noise_covariance, const double& outlier_thresh)
    {
      VectorXd residual = measurement - observation_model * estimate_state;
      MatrixXd inovation_covariance = observation_model * estimate_covariance * observation_model.transpose() + measurement_noise_covariance;
      return measurementOutlierCheck(residual, inovation_covariance, outlier_thresh);
    }

    void resetState()
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      predict_handlers_.clear();
      last_estimated_cov_handler_ = nullptr;
      init_state_ = VectorXd::Zero(state_dim_);
    }

    void setInitState(const double state_value, const int no)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      assert(no < init_state_.size());
      init_state_(no) = state_value;
    }

    inline void setInitState(const VectorXd init_state)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      assert(init_state_.size() == init_state.size());
      init_state_ = init_state;
    }

    virtual void setPredictionNoiseCovariance(const VectorXd& input_sigma_v)
    {
      MatrixXd input_sigma_m = (input_sigma_v).asDiagonal();
      input_noise_covariance_ = input_sigma_m * input_sigma_m;
    }

    inline const MatrixXd& getPredictionNoiseCovariance()
    {
      return input_noise_covariance_;
    }


    inline virtual void getMeasurementNoiseCovariance(const VectorXd& measure_sigma_v, MatrixXd& measurement_noise_covariance)
    {
      MatrixXd measure_sigma_m = (measure_sigma_v).asDiagonal();
      measurement_noise_covariance = measure_sigma_m * measure_sigma_m;
    }

    const VectorXd getEstimateState()
    {
      /* can not add const suffix for this function, because of the lock_guard */
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      if(predict_handlers_.size() > 0)
        return (predict_handlers_.back())->estimate_state_;
      else
        return init_state_; //no predction
    }

    inline const MatrixXd getEstimateCovariance()
    {
      /* TODO: this not correct, if we consider the delay (latency) in time_sync mode */
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      return last_estimated_cov_handler_->estimate_covariance_;
    }

    virtual void getPredictModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& state_transition_model, MatrixXd& control_input_model) const {};
    virtual void getCorrectModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& observation_model) const {};

    inline void setDebugVerbose(const bool flag)  { debug_verbose_ = flag; }

    inline void setInputFlag(bool flag = true) { input_start_flag_ = flag; }
    inline void setMeasureFlag(bool flag = true) { measure_start_flag_ = flag; }

    inline const bool getFilteringFlag() const
    {
      if(input_start_flag_ && measure_start_flag_) return true;
      else return false;
    }

    inline const int getStateDim() const {return state_dim_;}
    inline void setPredictBufSize(const int buf_size) {buf_size_ = buf_size;}
    inline const int getId() const {return id_;}

  protected:
    bool debug_verbose_;
    string name_;
    int id_;
    std::vector<string> state_names_;
    std::vector<string> input_names_;
    std::vector<string> measure_names_;

    VectorXd init_state_;
    PredictHandlerPtr last_estimated_cov_handler_;

    size_t buf_size_;
    std::deque<PredictHandlerPtr> predict_handlers_;

    /* TODO: constant variables regarding to input */
    int state_dim_;
    MatrixXd input_noise_covariance_;

    /* filtering start flag */
    bool input_start_flag_;
    bool measure_start_flag_;

    /* for mutex */
    std::recursive_mutex kf_mutex_;

    inline  const void statePropagation(const VectorXd& estimate_state,
                                        const VectorXd& input,
                                        const MatrixXd& state_transition_model,
                                        const MatrixXd& control_input_model,
                                        VectorXd& propagated_state) const
    {
      propagated_state = state_transition_model * estimate_state + control_input_model * input;
    }

    inline  const void covariancePropagation(const MatrixXd& estimate_covariance,
                                                 const MatrixXd& state_transition_model,
                                                 const MatrixXd& control_input_model,
                                                 const MatrixXd& input_noise_covariance,
                                                 MatrixXd& propagated_covariance) const
    {
      propagated_covariance =  state_transition_model * estimate_covariance * state_transition_model.transpose() + control_input_model * input_noise_covariance * control_input_model.transpose();
    }

    const bool correctionCore(const VectorXd& estimate_state,
                              const MatrixXd& estimate_covariance,
                              const VectorXd& measurement,
                              const MatrixXd& observation_model,
                              const MatrixXd& measurement_noise_covariance,
                              const double& outlier_thresh,
                              VectorXd& correct_state,
                              MatrixXd& correct_covariance)
    {
      MatrixXd inovation_covariance = observation_model * estimate_covariance * observation_model.transpose() + measurement_noise_covariance;
      MatrixXd kalman_gain = estimate_covariance * observation_model.transpose() * inovation_covariance.inverse();
      VectorXd residual = measurement - observation_model * estimate_state;

      /* check outlier */
      if(outlier_thresh > 0)
        {
          if(!measurementOutlierCheck(residual, inovation_covariance, outlier_thresh)) return false;
        }

      correct_state = estimate_state + kalman_gain * residual;
      correct_covariance = (MatrixXd::Identity(state_dim_, state_dim_) - kalman_gain * observation_model) * estimate_covariance;

      if(debug_verbose_)
        {
          cout << name_ << ", correct: " << endl;
          cout << "latest propagated state" << endl << correct_state << endl;
          cout << "last sync covariance" << endl << correct_covariance << endl;
          cout << "kalman_gain" << endl << kalman_gain  << endl;

          cout << "observation_model" << endl << observation_model  << endl;
          cout << "inovation_covariance" << endl << inovation_covariance  << endl;
        }
      return true;
    }


    /*
      - Simon Lynen, et.al,
      "A Robust and Modular Multi-Sensor Fusion Approach Applied to MAV Navigation"

      - Stephan Weiss, et.al,
      "Versatile Distributed Pose Estimation and Sensor
      Self-Calibration for an Autonomous MAV"
    */
    PredictHandlerPtr getTimeSyncPropagationResult(const double timestamp)
    {
      if(predict_handlers_.size() == 0)
        {
          if(debug_verbose_)
            cout << name_ << ", the predict_handlers is empty" << endl;
          return nullptr; //no valid data
        }

      if (timestamp > 0 && timestamp < predict_handlers_.front()->timestamp_)
        {
          if(predict_handlers_.size() < buf_size_) return predict_handlers_.front();

          if(debug_verbose_)
            {
              cout << name_ << ", the mesaure time stamp is older than the oldest predict handler: "
                   << std::fixed << std::setprecision(3) << timestamp << " vs "
                   << std::fixed << std::setprecision(3) << predict_handlers_.front()->timestamp_
                   << endl;
            }
          return nullptr; //no valid data
        }

      else if(timestamp > predict_handlers_.back()->timestamp_)
        {
          if(debug_verbose_)
            cout << name_ << ", the mesaure time stamp is future than the latest predict handler: " <<  timestamp << " vs " <<  predict_handlers_.back()->timestamp_ << endl;
          return nullptr; //no valid data
        }
      else
        {
          PredictHandlerPtr sync_predict_handler = nullptr;
          if(timestamp == -1)
            {
              sync_predict_handler = predict_handlers_.back(); // no time sync, use the lasted data
            }
          else
            {
              size_t candidate_index = (predict_handlers_.size() - 1) * (timestamp - predict_handlers_.front()->timestamp_) / (predict_handlers_.back()->timestamp_ - predict_handlers_.front()->timestamp_);

              if(timestamp > predict_handlers_.at(candidate_index)->timestamp_)
                {
                  if(debug_verbose_)
                    cout << name_ << ": start timestamp searching from index: " << candidate_index << endl;

                  for(auto it = predict_handlers_.begin() + candidate_index; it != predict_handlers_.end(); ++it)
                    {
                      /* future timestamp, escape */
                      if((*it)->timestamp_ > timestamp)
                        {
                          if (fabs((*it)->timestamp_ - timestamp) < fabs((*(it - 1))->timestamp_ - timestamp))
                            sync_predict_handler = *it;
                          else
                            sync_predict_handler = *(it - 1);
                          break;
                        }
                    }
                }
              else
                {
                  if(debug_verbose_)
                    cout << name_ << ": start timestamp reverse searching from index: " << candidate_index << endl;

                  for(auto it = predict_handlers_.rbegin() + (predict_handlers_.size() - 1 - candidate_index); it != predict_handlers_.rend(); ++it)
                    {
                      /* future timestamp, escape */
                      if((*it)->timestamp_ < timestamp)
                        {
                          if (fabs((*it)->timestamp_ - timestamp) < fabs((*(it - 1))->timestamp_ - timestamp))
                            sync_predict_handler = *it;
                          else
                            sync_predict_handler = *(it - 1);
                          break;
                        }
                    }
                }
              if(debug_verbose_)
                cout << name_ << ", sensor timestamp: " << std::fixed << std::setprecision(3) << timestamp
                     << ", imu timestamp: " << std::fixed << std::setprecision(3) << sync_predict_handler->timestamp_ << ", next imu timestamp: " << sync_predict_handler->next_handler_->timestamp_ << endl;


              assert(sync_predict_handler != nullptr);
            }

          if(last_estimated_cov_handler_->timestamp_ < sync_predict_handler->timestamp_)
            {
              /* propagate residual convariance */
              int cnt = 0;
              MatrixXd state_transition_model;
              MatrixXd control_input_model;
              while(1)
                {
                  cnt++;
                  MatrixXd prev_covariance = last_estimated_cov_handler_->estimate_covariance_;

                  assert(last_estimated_cov_handler_->next_handler_ != nullptr);
                  last_estimated_cov_handler_ = last_estimated_cov_handler_->next_handler_; //incremental
                  assert(last_estimated_cov_handler_->correct_handlers_.size() == 0);

                  getPredictModel(last_estimated_cov_handler_->model_params_,
                                  last_estimated_cov_handler_->estimate_state_,
                                  state_transition_model, control_input_model);
                  covariancePropagation(prev_covariance,
                                        state_transition_model, control_input_model,
                                        input_noise_covariance_,
                                        last_estimated_cov_handler_->estimate_covariance_);

                  if(last_estimated_cov_handler_ == sync_predict_handler) break;
                }
            }

          return sync_predict_handler;
        }
    }

    void rePropagation(const PredictHandlerPtr& sync_predict_handler)
    {
      if(sync_predict_handler == predict_handlers_.back())
        {
          return;
        }

      last_estimated_cov_handler_ = sync_predict_handler; // update the last estimated_cov_handler;

      double cov_update_laset_timestamp = sync_predict_handler->timestamp_;
      PredictHandlerPtr handler = sync_predict_handler->next_handler_;

      while(1)
        {
          /* record the last (future) handler which need to update the covariance */
          if(handler->correct_handlers_.size() > 0) last_estimated_cov_handler_ = handler;

          handler = handler->next_handler_;

          if(handler == nullptr) break;
        }

      handler = sync_predict_handler->next_handler_;
      VectorXd prev_state = sync_predict_handler->estimate_state_;
      MatrixXd prev_covariance = sync_predict_handler->estimate_covariance_;

      while(1)
        {
          /* propagate state */
          /* update the model */
          MatrixXd state_transition_model, control_input_model;
          getPredictModel(handler->model_params_, prev_state,
                          state_transition_model, control_input_model);
          statePropagation(prev_state, handler->input_,
                           state_transition_model, control_input_model,
                           handler->estimate_state_);

          /*  propagate of covariance */
          if(handler->timestamp_ <= last_estimated_cov_handler_->timestamp_)
            {
              covariancePropagation(prev_covariance,
                                    state_transition_model, control_input_model,
                                    input_noise_covariance_,
                                    handler->estimate_covariance_);

              /* correction if there is sensor measurement */
              for(auto it: handler->correct_handlers_)
                {
                  //cout << name_ << ": multi-sensor correction: " << std::fixed << std::setprecision(3) <<  it->timestamp_ <<endl;

                  VectorXd old_estimate_state = handler->estimate_state_;
                  MatrixXd old_estimate_covariance = handler->estimate_covariance_ ;
                  MatrixXd observation_model;
                  getCorrectModel(it->model_params_, old_estimate_state, observation_model);
                  correctionCore(old_estimate_state, old_estimate_covariance,
                                 it->measure_value_, observation_model,
                                 it->measurement_noise_covariance_,
                                 false,
                                 handler->estimate_state_,
                                 handler->estimate_covariance_);

                }
            }
          prev_state = handler->estimate_state_;
          prev_covariance = handler->estimate_covariance_;
          handler = handler->next_handler_;

          if(handler == nullptr) break;
        }
    }

    const bool measurementOutlierCheck(const VectorXd& residual, const MatrixXd& covariance, const double& outlier_thresh) const
    {
      double mahalonobis_dist_square = residual.transpose() * covariance.inverse() * residual;

      /*
        TODO:
        Table of Chi-Square Probabilities:
        https://people.richland.edu/james/lecture/m170/tbl-chi.html
      */
      if(mahalonobis_dist_square > outlier_thresh)
        {
          if(debug_verbose_)
            {
              cout << name_ << ", measurement is outlier: " << mahalonobis_dist_square << " vs " << outlier_thresh << endl;
              cout << name_ << ", residual" << endl <<  residual << endl;
              cout << name_ << ", mahalonobis_dist_square" << endl <<  mahalonobis_dist_square << endl;
              cout << name_ << ", covariance" << endl <<  covariance << endl;
            }
          return false;
        }
      else
        {
          return true;
        }
    }
  };
};
#endif
