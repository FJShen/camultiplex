//
// Created by nvidia on 10/20/19.
//


#include <ros/ros.h>
#include "camultiplex/my_nodes.h"

int main(int argc, char** argv){

    std::cout<<"independent drain node started";

    ros::init(argc, argv, "independent_drain_node");

    camera::drain_independent drain_node;

    ros::spin();

}