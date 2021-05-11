// -*- mode: c++ -*-
//pcl
#include <pcl/conversions.h> // pcl::console
#include "jsk_footstep_planner/grid_path_planner.h"

int main(int argc, char** argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  ros::init(argc, argv, "grid_path_planner");
  ros::NodeHandle pnh("~");

  jsk_footstep_planner::GridPathPlanner planner(pnh);

  ros::spin();
}
