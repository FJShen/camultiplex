#include "camultiplex/helper.h"
#include <boost/thread.hpp>

using namespace helper;

void Counter::updateSeq(unsigned long value) {
    //the Counter is an unique resource shared by many threads (from drain node) that have access to it
    //it is imperative to use a mutex to ensure thread safety of any write operation to the member variables of this class
    boost::unique_lock<boost::mutex> lock(counter_mutex);
    
    if (first_time) {
        current_seq = value;
        first_time = false;
        return;
    } else {
        loss += (long(value) - long(current_seq) - 1);
        previous_seq = current_seq;
        current_seq = value;
        return;
    }
    
    //upon function return, lock is automatically destroyed due to RAII. counter_mutex is implicitly released.
}

long int Counter::getLoss() {
    return loss;
}

unsigned long Counter::getCurrentSeq() {
    return current_seq;
}
