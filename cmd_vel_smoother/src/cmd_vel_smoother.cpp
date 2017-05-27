#include <cmath>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cmd_vel_smoother/CmdVelSmootherConfig.h>
#include <geometry_msgs/Twist.h>

template<class T> inline T sgn(T val) { return val > 0 ? 1.0 : -1.0; }

namespace dyn = dynamic_reconfigure;

class VelocitySmootherNode
{
  ros::NodeHandle nh_, pnh_;
  ros::Publisher  pub_;
  ros::Subscriber sub_;
  ros::Timer timer_;

  boost::mutex cfg_mutex_;
  typedef cmd_vel_smoother::CmdVelSmootherConfig Config;
  boost::shared_ptr<dyn::Server<Config> > dyn_srv_;
  
  int interpolate_max_frame_, count_;
  double desired_rate_;
  double x_acc_lim_, y_acc_lim_, yaw_acc_lim_;
  ros::Time latest_time_;
  geometry_msgs::Twist::ConstPtr latest_vel_;
  geometry_msgs::Twist::Ptr pub_vel_;

public:
  ros::Duration timerDuration() {
    return ros::Duration(1./(desired_rate_ * 2.2));
  }

  void dynConfigCallback(Config &cfg, uint32_t level) {
    boost::mutex::scoped_lock lock(cfg_mutex_);
    x_acc_lim_ = cfg.x_acc_lim;
    y_acc_lim_ = cfg.y_acc_lim;
    yaw_acc_lim_ = cfg.yaw_acc_lim;
    interpolate_max_frame_ = cfg.interpolate_max_frame;

    if(desired_rate_ != cfg.desired_rate) {
      desired_rate_ = cfg.desired_rate;
      if (timer_.isValid()) {
        ros::Duration d = timerDuration();
        timer_.setPeriod(d);
        ROS_INFO_STREAM("timer loop rate is changed to " << 1.0 / d.toSec() << "[Hz]");
      }
    }
  }

  void velCallback(const geometry_msgs::Twist::ConstPtr &msg){
    latest_time_ = ros::Time::now();
    latest_vel_ = msg;
    count_ = 0;
    pub_.publish(msg);
  }

  void timerCallback(const ros::TimerEvent& event){
    if (count_ > interpolate_max_frame_) return;
    if (latest_vel_ == NULL) return;

    double past_sec = (event.current_real - latest_time_).toSec();
    if (past_sec < 1.0 / desired_rate_) return;

    double g_x   = std::abs(latest_vel_->linear.x  / past_sec);
    double g_y   = std::abs(latest_vel_->linear.y  / past_sec);
    double g_yaw = std::abs(latest_vel_->angular.z / past_sec);

    ROS_DEBUG_STREAM("G: (" << g_x << ", " << g_y << ", " << g_yaw << ") detected");

    bool need_publish = false;
    if (g_x > x_acc_lim_) {
      pub_vel_->linear.x = (g_x - x_acc_lim_) * past_sec * sgn(latest_vel_->linear.x);
      need_publish = true;
    }

    if (g_y > y_acc_lim_) {
      pub_vel_->linear.y = (g_y - y_acc_lim_) * past_sec * sgn(latest_vel_->linear.y);
      need_publish = true;
    }

    if (g_yaw > yaw_acc_lim_) {
      pub_vel_->angular.z = (g_yaw - yaw_acc_lim_) * past_sec * sgn(latest_vel_->angular.z);
      need_publish = true;
    }

    if (need_publish)
      pub_.publish(*pub_vel_);
    count_++;
  }

  VelocitySmootherNode(): nh_(), pnh_("~") {
    latest_time_ = ros::Time::now();

    dyn_srv_ = boost::make_shared<dyn::Server<Config> >(pnh_);
    dyn::Server<Config>::CallbackType f = boost::bind(&VelocitySmootherNode::dynConfigCallback, this, _1, _2);
    dyn_srv_->setCallback(f);
    
    pub_vel_ = boost::shared_ptr<geometry_msgs::Twist>(new geometry_msgs::Twist());
    pub_ = nh_.advertise<geometry_msgs::Twist>("output", 100);
    sub_ = nh_.subscribe("input", 1, &VelocitySmootherNode::velCallback, this);
    timer_ = nh_.createTimer(timerDuration(), &VelocitySmootherNode::timerCallback, this);
  };
  ~VelocitySmootherNode() {};

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_vel_smoother");
  VelocitySmootherNode n;
  ros::spin();

  return 0;
}

