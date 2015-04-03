// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#ifndef JSK_FOOTSTEP_CONTROLLER_FOOTCOORDS_H_
#define JSK_FOOTSTEP_CONTROLLER_FOOTCOORDS_H_

#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <jsk_footstep_controller/GroundContactState.h>


namespace jsk_footstep_controller
{
  template <class T>
  class TimeStampedVector: public std::vector<T>
  {
  public:
    typedef typename std::vector<T>::iterator iterator;
    void removeBefore(const ros::Time& stamp)
    {
      for (iterator it = std::vector<T>::begin();
           it != std::vector<T>::end();) {
        if (((*it)->header.stamp - stamp) < ros::Duration(0.0)) {
          it = this->erase(it);
        }
        else {
          ++it;
        }
      }
    }
  protected:
  private:
  };

  
  class ValueStamped
  {
  public:
    typedef boost::shared_ptr<ValueStamped> Ptr;
    std_msgs::Header header;
    double value;
    ValueStamped(const std_msgs::Header& aheader, double avalue):
      header(aheader), value(avalue) {
    }
    ValueStamped() { }
  };

  class Footcoords
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    geometry_msgs::WrenchStamped,
    geometry_msgs::WrenchStamped> SyncPolicy;

    Footcoords();
    virtual ~Footcoords();

    enum SupportLegStatus
    {
      LLEG_GROUND, RLEG_GROUND, AIR, BOTH_GROUND, UNSTABLE
    };

  protected:
    
    // methods

    virtual void filter(const geometry_msgs::WrenchStamped::ConstPtr& lfoot,
                        const geometry_msgs::WrenchStamped::ConstPtr& rfoot);
    virtual bool computeMidCoords(const ros::Time& stamp);
    virtual bool computeMidCoordsFromSingleLeg(const ros::Time& stamp,
                                               bool use_left_leg);
    virtual bool waitForEndEffectorTrasnformation(const ros::Time& stamp);
    virtual bool waitForSensorFrameTransformation(const ros::Time& stamp,
                                                  const std::string& lsensor_frame,
                                                  const std::string& rsensor_frame);
    virtual bool updateGroundTF();
    virtual void publishTF(const ros::Time& stamp);
    virtual void publishState(const std::string& state);
    virtual void updateLegDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void publishContactState(const ros::Time& stamp);
    virtual double applyLowPassFilter(double current_val, double prev_val) const;
    virtual bool allValueLargerThan(TimeStampedVector<ValueStamped::Ptr>& values,
                                    double threshold);
    virtual bool allValueSmallerThan(TimeStampedVector<ValueStamped::Ptr>& values,
                                     double threshold);
    virtual bool resolveForceTf(const geometry_msgs::WrenchStamped::ConstPtr& lfoot,
                                const geometry_msgs::WrenchStamped::ConstPtr& rfoot,
                                tf::Vector3& lfoot_force, tf::Vector3& rfoot_force);
    virtual void periodicTimerCallback(const ros::TimerEvent& event);
    // ros variables
    boost::mutex mutex_;
    ros::Timer periodic_update_timer_;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_lfoot_force_;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_rfoot_force_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    ros::Publisher pub_state_;
    ros::Publisher pub_contact_state_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    // parameters
    std::string output_frame_id_;
    std::string parent_frame_id_;
    std::string midcoords_frame_id_;
    SupportLegStatus support_status_;
    double force_thr_;
    bool before_on_the_air_;
    std::string lfoot_frame_id_;
    std::string rfoot_frame_id_;
    std::string lfoot_sensor_frame_;
    std::string rfoot_sensor_frame_;
    tf::Transform ground_transform_;
    tf::Transform midcoords_;
    tf::Transform locked_midcoords_to_odom_on_ground_;
    boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    
    double prev_lforce_;
    double prev_rforce_;
    double alpha_;
    double sampling_time_;
    TimeStampedVector<ValueStamped::Ptr> lforce_list_;
    TimeStampedVector<ValueStamped::Ptr> rforce_list_;
  private:
  };
}
#endif
