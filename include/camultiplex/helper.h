#ifndef PIPELINE_NODELET_2__HELPER_
#define PIPELINE_NODELET_2__HELPER_

#include <string>
#include <boost/thread.hpp>
#include <librealsense2/rs.hpp>

/**
 * \namespace helper
 * \brief This namespace provides auxiliary functionality for the drain and source to use
 *
 * It includes two functions that can get the current Unix-time, and a counter class that can track packet indexes and estimate packet loss
 * \see helper.cpp, counter.cpp
 */
namespace helper {
    
    /**
     * \brief Prints to stdout the current Unix-time (in microseconds).
     *
     * Prints to stdout a user supplied string followed by the current Unix-time (in microseconds). The output string ends with a '\\t'
     * @param comment The user-supplied string
     */
    void print_time_stamp(std::string comment = "timestamp");
    
    /**
     * \brief Get current Unix-time in microseconds
     * \return Time in microseconds
     */
    std::string get_time_stamp_str();
    
    void print_intrinsics(rs2_intrinsics const&, std::string s = "");
    void print_extrinsics(rs2_extrinsics const&, std::string s = "");
    
    /**
     * \class Counter helper.h
     * \brief Keeps track of the index of the images being received by the drain.
     *
     * The drain instance shall encapsulate two counters, one for the RGB subscriber and one for the depth subscriber.
     * Upon receiving a new image from the source, the subscriber thread will call Counter's updateSeq method and pass it the image's index.
     * Counter will compare the current index with the previous index and calculate the number of lost images during transmission
     *
     * \ref Counter is thread safe.
     */
    class Counter {
    public:
        
        ///\brief Constructor of counter
        Counter() : first_time(true), loss(0), current_seq(0),
                    previous_seq(0) {} /**<Initializes loss, current_seq and previous_seq to zero*/
        
        /**
         * \brief Get the estimated value of lost images/messages
         * @return \ref loss
         * \see updateSeq
         */
        long int getLoss();
        
        /**
         * \brief Get the currently stored index value
         * @return \ref current_seq
         */
        unsigned long getCurrentSeq();
        
        /**
         * \brief updateSeq
         *
         * Everytime this method is called, \ref loss will be updated based on the difference between current_seq and previous_seq.
         *
         * Normally, this difference is expected to be 1, which means no loss between the arrival of two images/messages. In this case, \ref loss does not change.
         *
         * If the diff. > 1, at least one message is not received yet, and \ref loss will increase.
         *
         * @param index Index of the newly received image/message
         */
        void updateSeq(unsigned long index);
    
    private:
        unsigned long current_seq;
        unsigned long previous_seq;
        long int loss;
        bool first_time; /**<If \ref updateSeq is being called for the first time, this bool value would be true and prevent \ref loss from being incremented*/
        boost::mutex counter_mutex; /**<\ref Counter is designed to be accessible to the many subscriber threads managed by the drain. A mutex is essential to protect it against multi-thread conflicts*/
    };
    
}

#endif
