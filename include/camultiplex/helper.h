#ifndef PIPELINE_NODELET_2__HELPER_
#define PIPELINE_NODELET_2__HELPER_

#include <chrono>

namespace helper{
    
    inline void print_time_stamp(std::string comment="timestamp"){
	std::cout<< comment<<" "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()<<"\t";
    };
    
    inline std::string get_time_stamp_str(){
	std::stringstream ss;
	ss << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	return ss.str();
    }


}

#endif
