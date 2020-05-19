#include "camultiplex/helper.h"
#include <boost/thread.hpp>

namespace helper {
    
    void counter::updateSeq(unsigned long value) {
        //the counter is an unique resource shared by many threads (from drain node) that have access to it
        //it is imperative to use a mutex to ensure thread safety of any write operation to the member variables of this class
        boost::unique_lock<boost::mutex> lock(counter_mutex);
        
        if (first_time) {
            current_seq = value;
            first_time = false;
            return;
        } else {
            previous_seq = current_seq;
            current_seq = value;
            loss += (current_seq - previous_seq - 1);
            return;
        }
        
        //upon function return, lock is automatically destroyed due to RAII. counter_mutex is implicitly released.
    }
    
    long int counter::getLoss() {
        return loss;
    }
    
    unsigned long counter::getCurrentSeq() {
        return current_seq;
    }
    
    
}
