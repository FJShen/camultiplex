#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "camultiplex/my_nodes.h"

PLUGINLIB_EXPORT_CLASS(camera::source_nodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(camera::drain_nodelet, nodelet::Nodelet);
