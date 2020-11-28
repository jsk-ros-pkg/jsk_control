#include "jsk_footstep_controller/simple_footcoords.h"
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace jsk_footstep_controller{

  SimpleFootcoords::SimpleFootcoords():
    isActive_(true)
  {
    ros::NodeHandle nh, pnh("~");
    tfListener_.reset(new tf::TransformListener());

    pnh.param("lfoot_frame_id", lfoot_frame_id_,
              std::string("lleg_end_coords"));
    pnh.param("rfoot_frame_id", rfoot_frame_id_,
              std::string("rleg_end_coords"));

    odomSub_ = nh.subscribe("odom", 3, &SimpleFootcoords::odomCallback, this); // keep only latest ones to avoid latency because waitForTransform in callback function takes much time.
    odomBaseFootPrintPub_ = nh.advertise<nav_msgs::Odometry>("odom_base_footprint", 1);
    enableService_ = pnh.advertiseService("enable", &SimpleFootcoords::enableCallback, this);
  }

  void SimpleFootcoords::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    if (!isActive_) return;

    std::vector<geometry_msgs::TransformStamped> tf_transforms;

    // odom
    Eigen::Affine3d odom_pose;
    tf::poseMsgToEigen(msg->pose.pose, odom_pose);
    geometry_msgs::TransformStamped odom_coords;
    odom_coords.header = msg->header;
    odom_coords.child_frame_id = msg->child_frame_id;
    tf::transformEigenToMsg(odom_pose, odom_coords.transform);
    tf_transforms.push_back(odom_coords);
    tfBroadcaster_.sendTransform(tf_transforms); // odom tf is used below so odom tf need to be published here
    tf_transforms.clear();

    // base_footprint
    if(!waitForEndEffectorTransformation(msg->header.stamp)) {
      ROS_ERROR("[SimpleFootcoords::computeMidCoords] Failed to lookup endeffector transformation");
    } else {
      tf::StampedTransform lfoot_transform;
      tfListener_->lookupTransform("odom", lfoot_frame_id_, msg->header.stamp, lfoot_transform);
      Eigen::Affine3d lfoot_pose;
      tf::transformTFToEigen(lfoot_transform,lfoot_pose);

      tf::StampedTransform rfoot_transform;
      tfListener_->lookupTransform("odom", rfoot_frame_id_, msg->header.stamp, rfoot_transform);
      Eigen::Affine3d rfoot_pose;
      tf::transformTFToEigen(rfoot_transform,rfoot_pose);

      Eigen::Affine3d base_footprint_pose;
      base_footprint_pose.translation()(0) = (rfoot_pose.translation()(0) + lfoot_pose.translation()(0)) / 2.0;
      base_footprint_pose.translation()(1) = (rfoot_pose.translation()(1) + lfoot_pose.translation()(1)) / 2.0;
      base_footprint_pose.translation()(2) = std::min(rfoot_pose.translation()(2), lfoot_pose.translation()(2));

      Eigen::Quaterniond lfoot_quat(lfoot_pose.linear());
      Eigen::Quaterniond rfoot_quat(rfoot_pose.linear());
      Eigen::Quaterniond mid_quat = lfoot_quat.slerp(0.5,rfoot_quat);
      Eigen::Matrix3d mid_rot = mid_quat.toRotationMatrix();
      // roll and pitch should be zero respect to odom
      Eigen::Vector3d axis = (mid_rot*Eigen::Vector3d::UnitZ()).cross(Eigen::Vector3d::UnitZ());
      if(axis.norm()!=0){
        double sin = axis.norm();
        double cos = (mid_rot*Eigen::Vector3d::UnitZ()).dot(Eigen::Vector3d::UnitZ());
        double angle = std::atan2(sin,cos);
        mid_rot = (Eigen::AngleAxisd(angle,axis.normalized()).toRotationMatrix() * mid_rot).eval();
      }
      base_footprint_pose.linear() = mid_rot;

      geometry_msgs::TransformStamped base_footprint_coords;
      base_footprint_coords.header = msg->header;
      base_footprint_coords.header.frame_id = "odom";
      base_footprint_coords.child_frame_id = "base_footprint";
      tf::transformEigenToMsg(base_footprint_pose, base_footprint_coords.transform);
      tf_transforms.push_back(base_footprint_coords);

      nav_msgs::Odometry base_footprint_msg;
      base_footprint_msg.header = msg->header;
      base_footprint_msg.header.frame_id = "odom";
      base_footprint_msg.child_frame_id = "base_footprint";
      tf::poseEigenToMsg(base_footprint_pose,base_footprint_msg.pose.pose);
      odomBaseFootPrintPub_.publish(base_footprint_msg);
    }

    tfBroadcaster_.sendTransform(tf_transforms);
  }

  bool SimpleFootcoords::enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    if(isActive_ == request.data){
      ROS_INFO("[SimpleFootcoords::enableCallback] Already %s",(isActive_ ? "Enabled" : "Disabled"));
      response.success = true;
      response.message = "";
      return true;
    } else {
      isActive_ = request.data;

      ROS_INFO("[SimpleFootcoords::enableCallback] %s",(isActive_ ? "Enabled" : "Disabled"));
      response.success = true;
      response.message = "";
      return true;
    }
  }

  bool SimpleFootcoords::waitForEndEffectorTransformation(const ros::Time& stamp)
  {
    // root -> lfoot
    if (!tfListener_->waitForTransform("odom", lfoot_frame_id_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("[SimpleFootcoords::waitForEndEffectorTrasnformation] failed to lookup transform between %s and %s",
                "odom",
                lfoot_frame_id_.c_str());
      return false;
    }
    // root -> rfoot
    else if (!tfListener_->waitForTransform("odom", rfoot_frame_id_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("[SimpleFootcoords::waitForEndEffectorTrasnformation]failed to lookup transform between %s and %s",
                "odom",
                rfoot_frame_id_.c_str());
      return false;
    }
    return true;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_footcoords");
  jsk_footstep_controller::SimpleFootcoords c;
  ros::AsyncSpinner spinner(10); // Use many threads to increase frequency because waitForTransform in callback function takes much time.
  spinner.start();
  ros::waitForShutdown();
}
