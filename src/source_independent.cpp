#include "camultiplex/my_nodes.h"
#include "ros/ros.h"
#include <boost/thread.hpp>
#include <iostream>
#include <vector>

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

void source_independent::timerCallback(const ros::TimerEvent &event) {
    
    //define lambda which will be launched in parallel
    auto f = [&]() {
        try {
            while (1) { this->parallelAction(); }
        }
        catch (boost::thread_interrupted &) {
            return;
        }
    };
    
    for (int i = 0; i < N; i++) {
        thread_list.emplace_back(f);
    }
}