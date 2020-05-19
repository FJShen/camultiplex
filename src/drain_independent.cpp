#include "camultiplex/my_nodes.h"
#include "ros/ros.h"
#include <iostream>

using namespace camera;

drain_independent::~drain_independent() {
    ros::NodeHandle& rhp = getMyPrivateNodeHandle();
    rhp.deleteParam("diversity");
    rhp.deleteParam("base_path");
}

ros::NodeHandle& drain_independent::getMyNodeHandle() {
    return nh;
}

ros::NodeHandle& drain_independent::getMyPrivateNodeHandle() {
    return nph;
}

void drain_independent::selfInit() {
    nph = ros::NodeHandle(ros::this_node::getName()); //assign the name of this node to nph, everything that nph have access to is in the private namespace of this node
    initialize();
    std::cout << ("Camera independent drain node selfInit called\n");
}