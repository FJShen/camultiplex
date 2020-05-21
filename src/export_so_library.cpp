#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "camultiplex/my_nodes.h"

PLUGINLIB_EXPORT_CLASS(camera::Source_nodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(camera::Drain_nodelet, nodelet::Nodelet);
