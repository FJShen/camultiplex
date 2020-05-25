#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "camultiplex/helper.h"
#include "camultiplex/my_nodes.h"
#include <nodelet/nodelet.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <ctime>

namespace camera {
    
    Drain_base::~Drain_base() {
        delete[] depth_sub;
        delete[] rgb_sub;
        std::cout << ("camera drain node base destructed\n");
    }
    
    
    void Drain_base::initialize() {
        create_directories();
        define_subscribers();
        
        ros::NodeHandle& handle = getMyNodeHandle();
        
        timer = handle.createTimer(ros::Duration(5), &Drain_base::timerCallback, this);
    }
    
    
    Drain_base& Drain_base::create_directories() {
        
        ros::NodeHandle& private_handle = getMyPrivateNodeHandle();
        
        if (!private_handle.getParam("base_path", base_path)) {
            std::cout << "No parameter for image base path specified, will use default: " << base_path << "\n";
        } else {
            std::cout << "Image base path: " << base_path << "\n";
        }
        
        //get the year_month_day_hour_min of this moment
        std::time_t t = std::time(nullptr);
        std::tm* local_time = std::localtime(&t);
        
        std::stringstream ss;
        ss << "/" << 1900 + (local_time->tm_year) << "_" << 1 + (local_time->tm_mon) << "_" << local_time->tm_mday
           << "_" << local_time->tm_hour << "_" << local_time->tm_min;
        
        //create directory
        boost::filesystem::path P{base_path};
        P += ss.str();
        boost::filesystem::create_directories(P);
        
        //set this as "current" directory
        boost::filesystem::current_path(P);
        
        boost::filesystem::create_directories(std::string("rgb_images"));
        boost::filesystem::create_directories(std::string("depth_images"));
        
        folder_path = P.string();
        std::cout << "Folder path is set to " << folder_path << "\n";
        
        return *this;
    }
    
    
    Drain_base& Drain_base::define_subscribers() {
        ros::NodeHandle& handle = getMyNodeHandle();
        ros::NodeHandle& private_handle = getMyPrivateNodeHandle();
        
        if (!private_handle.getParam("diversity", diversity)) {
            std::cout << "No parameter for drain channel diversity specified, will use default value: " << diversity
                      << "\n";
        } else {
            std::cout << "Number of drain channel diversity: " << diversity << "\n";
        }
        
        depth_sub = new ros::Subscriber[diversity];
        rgb_sub = new ros::Subscriber[diversity];
        
        //define subscribers
        const std::string s_d("depth");
        const std::string s_rgb("RGB");
        for (int i = 0; i < diversity; i++) {
            depth_sub[i] = handle.subscribe<sensor_msgs::Image>
                    (s_d + std::to_string(i), 1,
                     boost::bind(&Drain_base::drain_depth_callback, this, _1, i));
            rgb_sub[i] = handle.subscribe<sensor_msgs::Image>
                    (s_rgb + std::to_string(i), 1,
                     boost::bind(&Drain_base::drain_rgb_callback, this, _1, i));
        }
        
        std::cout << "Defined " << diversity << " subscribers\n";
        
        return *this;
    }
    
    
    void Drain_base::timerCallback(const ros::TimerEvent& event) {
        std::cout << "Drain: Depth received " << depth_counter.getCurrentSeq()
                  << " frames, total transmission loss estimate is "
                  << depth_counter.getLoss() << " frames\n";
        std::cout << "Drain: RGB received " << rgb_counter.getCurrentSeq()
                  << " frames, total transmission loss estimate is "
                  << rgb_counter.getLoss() << " frames\n";
    }
    
    
    //todo: is it necessary to distinguish callback functions by passing in distinctive index numbers?
    //callback to handle depth frame messages
    void Drain_base::drain_depth_callback(const sensor_msgs::Image::ConstPtr& msg, int channel_num) {
        
        //By calling toCvShare we are obtaining a read-only reference to the OpenCV Mat data in the received message.
        //If in the future, this Mat need to be *mutated*, we have to use toCvCopy instead of toCvShare.
        cv_bridge::CvImageConstPtr cv_const_ptr = cv_bridge::toCvShare(msg);
        
        boost::async(
                boost::bind(&Drain_base::save_image,
                            this,
                            cv_const_ptr,
                            boost::ref(msg->header),
                            DEPTH)
        );
        
        depth_counter.updateSeq(std::stoul(msg->header.frame_id));
    }
    
    
    //callback to handle rgb frame messages
    void Drain_base::drain_rgb_callback(const sensor_msgs::Image::ConstPtr& msg, int channel_num) {
        
        cv_bridge::CvImageConstPtr cv_const_ptr = cv_bridge::toCvShare(msg, "bgr8");

        boost::async(
                boost::bind(&Drain_base::save_image,
                            this,
                            cv_const_ptr,
                            boost::ref(msg->header),
                            RGB)
        );
        
        rgb_counter.updateSeq(std::stoul(msg->header.frame_id));
    }
    
    
    Drain_base&
    Drain_base::save_image(cv_bridge::CvImageConstPtr cv_const_ptr, std_msgs::Header header, unsigned int channel) {
        
        //name the image with the timestamp obtained from the source
        unsigned int seconds = header.stamp.sec;
        unsigned int nanoseconds = header.stamp.nsec;
        std::stringstream ss;
        ss << std::setw(10) << std::setfill('0') << seconds << "." << std::setw(9) << std::setfill('0') << nanoseconds;
        
        std::string myStr;
        
        switch (channel) {
            case RGB:
                myStr = this->folder_path + std::string("/rgb_images/") + ss.str() + ".jpg";
                break;
            
            case DEPTH:
                myStr = this->folder_path + std::string("/depth_images/") + ss.str() + ".png";
                break;
            
            default:
                std::cout << "save_image: channel type is neither depth nor rgb, re-check your code!\n";
                break;
        }
        
        try {
            cv::imwrite(myStr, cv_const_ptr->image);
        }
        catch (cv_bridge::Exception& e) {
            std::cout << "cv_bridge exception: " << e.what() << "\n";
            return *this;
        }
        return *this;
    }
    
}
