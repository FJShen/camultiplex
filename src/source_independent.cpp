#include "camultiplex/my_nodes.h"
#include "ros/ros.h"
#include <iostream>

using namespace camera;

source_independent::~source_independent(){
    ros::NodeHandle &rhp = getMyPrivateNodeHandle();
    rhp.deleteParam("diversity");
    rhp.deleteParam("FPS");
    rhp.deleteParam("align");

    ros::NodeHandle &rh = getMyNodeHandle();
    rh.deleteParam("rs_start_time");
}

void source_independent::selfInit(){
    nph = ros::NodeHandle(ros::this_node::getName());
    initialize();
    std::cout << ("Camera independent source node selfInit called\n");
}

ros::NodeHandle& source_independent::getMyNodeHandle(){
    return nh;
}

ros::NodeHandle& source_independent::getMyPrivateNodeHandle(){
    return nph;
}
