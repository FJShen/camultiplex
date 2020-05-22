//
// Created by nvidia on 10/20/19.
//


#include <ros/ros.h>
#include "camultiplex/my_nodes.h"

int main(int argc, char** argv){

    std::cout<<"independent drain node started";

    ros::init(argc, argv, "independent_drain_node");

    camera::Drain_independent drain_node;

    //todo: consider using ros::MultiThreadedSpinner and eliminate the need to use boost::async in camera::Drain_base::drain_depth_callback (and drain_rgb_callback)
    ros::spin();

}