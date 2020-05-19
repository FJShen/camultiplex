#ifndef PIPELINE_NODELET_2__HELPER_
#define PIPELINE_NODELET_2__HELPER_

#include <string>
#include <boost/thread.hpp>

namespace helper {

    void print_time_stamp(std::string comment = "timestamp");

    std::string get_time_stamp_str();


    class counter {
    public:
        counter() : first_time(true), loss(0), current_seq(0), previous_seq(0) {}

        long int getLoss();

        unsigned long getCurrentSeq();

        void updateSeq(unsigned long);

    private:
        unsigned long current_seq;
        unsigned long previous_seq;
        long int loss;
        bool first_time;
        boost::mutex counter_mutex;
    };

}

#endif
