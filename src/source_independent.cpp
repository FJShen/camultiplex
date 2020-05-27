#include "camultiplex/my_nodes.h"
#include "ros/ros.h"
#include <iostream>

using namespace camera;

Source_independent::~Source_independent(){
    ros::NodeHandle &rhp = getMyPrivateNodeHandle();
    rhp.deleteParam("diversity");
    rhp.deleteParam("FPS");
    rhp.deleteParam("align");

    ros::NodeHandle &rh = getMyNodeHandle();
    rh.deleteParam("rs_start_time");
}

void Source_independent::selfInit(){
    nph = ros::NodeHandle(ros::this_node::getName());
    initialize();
//    std::cout << ("Camera Source_independent selfInit called\n");
}

ros::NodeHandle& Source_independent::getMyNodeHandle(){
    return nh;
}

ros::NodeHandle& Source_independent::getMyPrivateNodeHandle(){
    return nph;
}
