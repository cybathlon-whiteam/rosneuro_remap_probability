#include <ros/ros.h>
#include "rosneuro_remap_probability/RosneuroRemapProbability.hpp"

int main(int argc, char **argv) {
  // ros initialization
  ros::init(argc, argv, "rosneuro_remap_probability");

  rosneuro::RemapProbability remap_probability;
  if (!remap_probability.configure()) {
    ROS_ERROR("Failed to configure remap probability");
    return -1;
  }

  remap_probability.run();
  return 0;
}
