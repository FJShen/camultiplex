#include "camultiplex/helper.h"

namespace helper{

    void counter::updateSeq(unsigned long value){
	if(first_time){
	    current_seq = value;
	    first_time = false;
	    return;
	}
	else{
	    previous_seq = current_seq;
	    current_seq = value;
	    loss += (current_seq - previous_seq -1);
	    return;
	}
    }

    long int counter::getLoss(){
	return loss;
    }

    unsigned long counter::getCurrentSeq(){
	return current_seq;
    }




}
