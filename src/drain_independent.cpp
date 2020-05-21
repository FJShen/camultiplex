#include "camultiplex/my_nodes.h"
#include "ros/ros.h"
#include <iostream>

using namespace camera;

Drain_independent::~Drain_independent() {
    ros::NodeHandle& rhp = getMyPrivateNodeHandle();
    rhp.deleteParam("diversity");
    rhp.deleteParam("base_path");
}

ros::NodeHandle& Drain_independent::getMyNodeHandle() {
    return nh;
}

ros::NodeHandle& Drain_independent::getMyPrivateNodeHandle() {
    return nph;
}

void Drain_independent::selfInit() {
    nph = ros::NodeHandle(ros::this_node::getName()); //assign the name of this node to nph, everything that nph have access to is in the private namespace of this node
    initialize();
    std::cout << ("Camera independent drain node selfInit called\n");
}