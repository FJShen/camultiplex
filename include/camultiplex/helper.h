#ifndef PIPELINE_NODELET_2__HELPER_
#define PIPELINE_NODELET_2__HELPER_

#include <chrono>
#include <iostream>

namespace helper{
    
    void print_time_stamp(std::string comment="timestamp");
    
    std::string get_time_stamp_str();
    
 
    class counter{
    public:
    counter() : first_time(true), loss(0){ }
	unsigned long getLoss();
	unsigned long getCurrentSeq();
	void updateSeq(unsigned long);
    private:
	unsigned long current_seq;
	unsigned long previous_seq;
	unsigned long loss;
	bool first_time;
    };
    
}
    
#endif
