//
// Created by nvidia on 10/20/19.
//

#include <ros/ros.h>
#include "camultiplex/my_nodes.h"

int main(int argc, char** argv){

    std::cout<<"independent source node started";

    ros::init(argc, argv, "independent_source_node");

    camera::source_independent source_node;

    ros::spin();

}