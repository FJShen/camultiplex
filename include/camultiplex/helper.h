#ifndef PIPELINE_NODELET_2__HELPER_
#define PIPELINE_NODELET_2__HELPER_

#include <chrono>
#include <iostream>

namespace helper{
    
    void print_time_stamp(std::string comment="timestamp");
    
    std::string get_time_stamp_str();
    
}

#endif
