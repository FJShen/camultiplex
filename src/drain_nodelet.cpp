#include "camultiplex/my_nodes.h"
#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <iostream>

using namespace camera;

ros::NodeHandle& Drain_nodelet::getMyNodeHandle() {
    ///getMTNodeHandle allows the all publishers/subscribers to run on multiple threads in the thread pool of nodelet manager.
    ///\see The thread model for ROS nodelets:
    ///http://wiki.ros.org/nodelet#Threading_Model
    return getMTNodeHandle();
}

ros::NodeHandle& Drain_nodelet::getMyPrivateNodeHandle() {
    ///getMTPrivateNodeHandle allows the all publishers/subscribers to run on multiple threads in the thread pool of nodelet manager.
    ///\see The thread model for ROS nodelets:
    ///http://wiki.ros.org/nodelet#Threading_Model
    return getMTPrivateNodeHandle();
}

void Drain_nodelet::onInit(){
    initialize();
    std::cout<<("Camera drain node onInit called\n");
}

Drain_nodelet::~Drain_nodelet() {
    ros::NodeHandle &rhp = getMyPrivateNodeHandle();
    rhp.deleteParam("diversity");
    rhp.deleteParam("base_path");
}
