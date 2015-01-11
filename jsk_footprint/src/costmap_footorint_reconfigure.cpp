#include <ros/ros.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <pcl_ros/transforms.h>
#include <jsk_pcl_ros/BoundingBoxArray.h>
#include <jsk_pcl_ros/SetPointCloud2.h>
#include <pcl_ros/pcl_nodelet.h>

// pcl
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/foreach.hpp>
#include <sstream>

using namespace std;
typedef  pair<double, double> pdd;

class FootPrintParser{
public:

  FootPrintParser(){
  }

  std::string parse_to_string(pcl::PointCloud<pcl::PointXYZ>::Ptr values){
    std::string result_string="[";
    int counter = 0;
    BOOST_FOREACH( pcl::PointXYZ p, values->points ){
      std::stringstream s;
      if (counter != 0){
        result_string += ",";
      }
      s << "[" << p.x << ", " << p.y << "]";
      result_string += s.str();
    }
    result_string += "]";
    ROS_INFO("parse_to_string %s", result_string.c_str());
    return result_string;
  }


  std::string parse_to_string(vector<pair<double, double> > values){
    std::string result_string="[";
    int counter = 0;
    BOOST_FOREACH( pdd p, values ){
      std::stringstream s;
      if (counter != 0){
        result_string += ",";
      }
      s << "[" << p.first << ", " << p.second << "]";
    }
    result_string += "]";
    ROS_INFO("parse_to_string %s", result_string.c_str());
    return result_string;
  }

  vector<pair<double, double> > parse_to_vector(std::string footprint){
    vector<pair<double, double> > result_vals_;
    vector<string> v;
    vector<double> prev_;

    boost::algorithm::split( v, footprint, boost::is_any_of(","));
    BOOST_FOREACH(string str, v){
      str.erase(boost::remove_if(str, boost::is_any_of("[] ")), str.end());
      double tmp = atof( str.c_str() );
      if(prev_.size() == 1){
        result_vals_.push_back(pair<double, double>( prev_[0], tmp ));
        prev_.clear();
      }else{
        prev_.push_back(tmp);
      }
    }
    return result_vals_;
  }

  //We assumes the z value is 0
  pcl::PointCloud<pcl::PointXYZ> parse_to_points(std::string footprint){
    pcl::PointCloud<pcl::PointXYZ> result_points_;
    vector<string> v;
    vector<double> prev_;

    boost::algorithm::split( v, footprint, boost::is_any_of(","));
    BOOST_FOREACH(string str, v){
      str.erase(boost::remove_if(str, boost::is_any_of("[] ")), str.end());
      double tmp = atof( str.c_str() );
      if(prev_.size() == 1){
        pcl::PointXYZ tmp_point(prev_[0], tmp, 0);
        result_points_.push_back(tmp_point);
        prev_.clear();
      }else{
        prev_.push_back(tmp);
      }
    }
    return result_points_;
  }

};

class CostmapFootPrintReconfigure{
public:
  CostmapFootPrintReconfigure():target_cloud_(new pcl::PointCloud<pcl::PointXYZ>),n_("~"), basic_coordiname_tf_("base_footprint"){

    if (!n_.getParam("consider_robotarms", consider_robotarms_))
      {
        ROS_WARN("~consider_robotarms is not specified");
        consider_robotarms_ = true;
      }

    if (!n_.getParam("consider_cloud", consider_cloud_))
      {
        ROS_WARN("~consider_cloud is not specified");
        consider_cloud_ = false;
      }

    if (!n_.getParam("default_footprint_string", default_footprint_string_))
      {
        ROS_WARN("~default_footprint_string is not specified");
        default_footprint_string_ = std::string("[[-0.34,-0.34],[-0.44,-0.19],[-0.44,0.19],[-0.34,0.34],[0.34,0.34],[0.39,0],[0.34,-0.34]]");
      }

    if (!n_.getParam("box_offset", box_offset_))
      {
        ROS_WARN("~box_offset is not specified");
        box_offset_ = 0;
      }

    if (!n_.getParam("height", height_))
      {
        ROS_WARN("~height is not specified");
	height_ = 2.0;
      }

    if (!n_.getParam("frame_id", frame_id_))
      {
        ROS_WARN("~frame_id is not specified");
	frame_id_ = "base_footprint";
      }

    if (!n_.getParam("enable_callrequest", enable_callrequest_))
      {
        ROS_WARN("~enable_callrequest is not specified");
	enable_callrequest_ = true;
      }

    if (!n_.getParam("enable_box_publish", enable_box_publish_))
      {
        ROS_WARN("~enable_box_publish is not specified");
	enable_box_publish_ = true;
      }

    //parse if consifer_robotarms is true
    if(consider_robotarms_){
      std::string tmp_tf_names;
      if (!n_.getParam("tf_names", tmp_tf_names))
        {
          ROS_WARN("~tf_names is not specified");
          tmp_tf_names = std::string("/larm,/rarm");
        }
      boost::algorithm::split( tf_names_, tmp_tf_names, boost::is_any_of(","));
    }
    
    srv_ = n_.advertiseService("reconfigure_costmap_footprint", &CostmapFootPrintReconfigure::request_cloud, this);
    sub_ = n_.subscribe("input", 1, &CostmapFootPrintReconfigure::cloud_cb,this);
    if(enable_box_publish_){
      pub_ = n_.advertise<jsk_pcl_ros::BoundingBoxArray>("boxes", 1);
      pub2_ = n_.advertise<jsk_pcl_ros::BoundingBox>("box", 1);
    }
  }

  void
  cloud_cb (const sensor_msgs::PointCloud2 &pc){
    ROS_INFO("cloud_cb");
    convert_and_transform(pc);
  }

  bool request_cloud(jsk_pcl_ros::SetPointCloud2::Request &req,
                     jsk_pcl_ros::SetPointCloud2::Response &res){
    ROS_INFO("request_cloud");
    convert_and_transform(req.cloud);
  }

  void convert_and_transform(const sensor_msgs::PointCloud2 &pc){
    static tf::TransformListener tf_listener_;
    sensor_msgs::PointCloud2 output;

    //Convert it to the base_footprint coordinates
    pcl_ros::transformPointCloud( basic_coordiname_tf_, pc, output, tf_listener_);
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(output, *target_cloud_);
    std::vector<int> indices;
    target_cloud_->is_dense = false;
    pcl::removeNaNFromPointCloud(*target_cloud_, *target_cloud_, indices);
    //if cloud is empty , set default parameter
    footprint_reconfigure();
  }

  void not_convert_and_transform(){
    //Convert it to the base_footprint coordinates
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    //if cloud is empty , set default parameter
    footprint_reconfigure();
  }

  void footprint_reconfigure(){
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::StrParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "footprint";
    if(target_cloud_->points.size() == 0 && consider_cloud_){
      ROS_INFO("Set default footprint %s", default_footprint_string_.c_str());
      double_param.value = default_footprint_string_;
    }else{
      std::string result_string = calcConvexHull();
      ROS_INFO("Set calculated footprint %s", result_string.c_str());
      double_param.value = result_string;
    }
    conf.strs.push_back(double_param);
    if(enable_callrequest_){
      srv_req.config = conf;
      ros::service::call("/move_base_node/local_costmap/set_parameters", srv_req, srv_resp);
    }
  }

  std::string calcConvexHull()
  {
    //get the points of default robot base footprint
    FootPrintParser fpp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr default_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(fpp.parse_to_points(default_footprint_string_), *default_points);

    //project and get convexHull of the given cloud
    //projection
    BOOST_FOREACH(pcl::PointXYZ point , target_cloud_->points){
      pcl::PointXYZ tmp_point;
      tmp_point.x = point.x;
      tmp_point.y = point.y;
      tmp_point.z = 0;
      default_points->points.push_back(tmp_point);
    }

    //If true add robot arm joins points
    if(consider_robotarms_){
      static tf::TransformListener tf_listener;
      BOOST_FOREACH(std::string tf_name , tf_names_){
        try{
          tf::StampedTransform transform;
          tf_listener.waitForTransform( basic_coordiname_tf_, tf_name, ros::Time(0), ros::Duration(0.5) );
          tf_listener.lookupTransform( basic_coordiname_tf_, tf_name, ros::Time(0), transform);
          pcl::PointXYZ tmp_point;
          tmp_point.x = transform.getOrigin().x();
          tmp_point.y = transform.getOrigin().y();
          tmp_point.z = 0;//transform.getOrigin().z();
          default_points->points.push_back(tmp_point);
        } catch (tf::TransformException ex) {
          ROS_INFO("%s",ex.what());
        }
      }
    }

    //convexHull
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ConcaveHull<pcl::PointXYZ> chull;

    chull.setAlpha(10);
    chull.setInputCloud (default_points);
    chull.reconstruct (*cloud_hull);

    //Publish box
    if(enable_box_publish_){
      jsk_pcl_ros::BoundingBoxArray bounding_box_array;
      bounding_box_array.header.stamp = ros::Time::now();
      bounding_box_array.header.frame_id = frame_id_;
      jsk_pcl_ros::BoundingBox bounding_box;
      Eigen::Vector4f minpt, maxpt;
      pcl::getMinMax3D<pcl::PointXYZ>(*cloud_hull, minpt, maxpt);
      double xwidth = maxpt[0] - minpt[0];
      double ywidth = maxpt[1] - minpt[1];
      double zwidth = maxpt[2] - minpt[2];
      Eigen::Vector4f center2((maxpt[0] + minpt[0]) / 2.0, (maxpt[1] + minpt[1]) / 2.0, (maxpt[2] + minpt[2]) / 2.0, 1.0);
      bounding_box.header.frame_id =frame_id_;
      bounding_box.header.stamp = ros::Time::now();
    
      bounding_box.pose.position.x = center2[0];
      bounding_box.pose.position.y = center2[1];
      bounding_box.pose.position.z = height_ / 2;
      bounding_box.pose.orientation.x = 0;
      bounding_box.pose.orientation.y = 0;
      bounding_box.pose.orientation.z = 0;
      bounding_box.pose.orientation.w = 1;
      bounding_box.dimensions.x = xwidth + box_offset_ * 2;
      bounding_box.dimensions.y = ywidth + box_offset_ * 2;
      bounding_box.dimensions.z = height_;

      pub2_.publish(bounding_box);
      bounding_box_array.boxes.push_back(bounding_box);
      pub_.publish(bounding_box_array);
    }

    return fpp.parse_to_string(cloud_hull);
  }

  void run(){
    ROS_INFO("recongifure_costmap_footprint's service server starts");
    while(ros::ok()){
      if(consider_cloud_){
        ros::spin();
      }else{
        not_convert_and_transform();
        boost::this_thread::sleep (boost::posix_time::milliseconds (200));
      }
    }
  }

  ros::NodeHandle n_;
  ros::ServiceServer srv_;
  ros::Publisher pub_;
  ros::Publisher pub2_;
  std::string default_footprint_string_;
  std::string frame_id_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  ros::Subscriber sub_;
  bool consider_robotarms_;
  bool consider_cloud_;
  bool enable_callrequest_;
  bool enable_box_publish_;
  double height_;
  double box_offset_;
  std::vector<std::string> tf_names_;
  std::string basic_coordiname_tf_;
};

int main(int argc, char* argv[]){
  ros::init(argc, argv, "costmap_footprint_reconfigure");

  CostmapFootPrintReconfigure c;
  c.run();
}
