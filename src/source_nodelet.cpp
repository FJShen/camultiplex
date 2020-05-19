#include "camultiplex/my_nodes.h"
#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <iostream>

namespace camera{
    
    ros::NodeHandle& source_nodelet::getMyNodeHandle(){
        return getMTNodeHandle();
    }
    
    ros::NodeHandle& source_nodelet::getMyPrivateNodeHandle(){
        return getMTPrivateNodeHandle();
    }
    
    void source_nodelet::onInit(){
        initialize();
        std::cout << ("Camera source node nodelet onInit called\n");
    }
    
    source_nodelet::~source_nodelet(){
        ros::NodeHandle &rhp = getMyPrivateNodeHandle();
        rhp.deleteParam("diversity");
        rhp.deleteParam("FPS");
        rhp.deleteParam("align");
        
        ros::NodeHandle &rh = getMyNodeHandle();
        rh.deleteParam("rs_start_time");
    }
    
}
