#include "camultiplex/my_nodes.h"
#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <iostream>

using namespace camera;

ros::NodeHandle& drain_nodelet::getMyNodeHandle() {
    return getMTNodeHandle();
}

ros::NodeHandle& drain_nodelet::getMyPrivateNodeHandle() {
    return getMTPrivateNodeHandle();
}

void drain_nodelet::onInit(){
    initialize();
    std::cout<<("Camera drain node onInit called\n");
}

drain_nodelet::~drain_nodelet() {
    ros::NodeHandle &rhp = getMyPrivateNodeHandle();
    rhp.deleteParam("diversity");
    rhp.deleteParam("base_path");
}
