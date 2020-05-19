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

    drain_base::~drain_base() {
        delete[] depth_sub;
        delete[] rgb_sub;
        std::cout << ("camera drain node base destructed\n");
    }


    //callback to handle depth frame messages
    void drain_base::drain_depth_callback(const sensor_msgs::Image::ConstPtr &msg, int channel_num) {

        //print debug information
//	NODELET_DEBUG_STREAM( "received depth frame"
//			      << (msg->header.frame_id) << " from channel "
//			      << std::to_string(channel_num));
//        std::cout << "received depth frame"
//                  << (msg->header.frame_id) << " from channel "
//                  << std::to_string(channel_num) << "\n";

        //By calling toCvShare we are obtaining a read-only reference to the OpenCV Mat data in the received message
        //If in the future, this Mat need to be mutated, we have to use toCvCopy instead of toCvShare
        //cv_bridge::CvImageConstPtr cv_const_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
        cv_bridge::CvImageConstPtr cv_const_ptr = cv_bridge::toCvShare(msg);

        boost::async(
                boost::bind(&drain_base::save_image,
                            this,
                            cv_const_ptr,
                            boost::ref(msg->header),
                            DEPTH)
        );

        depth_counter.updateSeq(std::stoul(msg->header.frame_id));

        //print debug information
//	NODELET_DEBUG_STREAM("saved depth frame" << (msg->header.frame_id));
//        std::cout << "saved depth frame" << (msg->header.frame_id) << "\n";
    }


    //callback to handle rgb frame messages
    void drain_base::drain_rgb_callback(const sensor_msgs::Image::ConstPtr &msg, int channel_num) {

//        NODELET_DEBUG_STREAM( "received rgb frame"
//			      << (msg->header.frame_id) << " from channel "
//			      << std::to_string(channel_num));
//        std::cout << "received rgb frame"
//                  << (msg->header.frame_id) << " from channel "
//                  << std::to_string(channel_num) << "\n";

        cv_bridge::CvImageConstPtr cv_const_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

        boost::async(
                boost::bind(&drain_base::save_image,
                            this,
                            cv_const_ptr,
                            boost::ref(msg->header),
                            RGB)
        );

        rgb_counter.updateSeq(std::stoul(msg->header.frame_id));

//        NODELET_DEBUG_STREAM("saved rgb frame" << (msg->header.frame_id));
//        std::cout << "saved rgb frame" << (msg->header.frame_id) << "\n";
    }
    
    drain_base &
    drain_base::save_image(cv_bridge::CvImageConstPtr cv_const_ptr, std_msgs::Header header, unsigned int channel) {

        //name the image with the timestamp obtained from the source
        unsigned int seconds = header.stamp.sec;
        unsigned int nanoseconds = header.stamp.nsec;
        std::stringstream ss;
        ss << std::setw(10) << std::setfill('0') << seconds << "." << std::setw(9) << std::setfill('0') << nanoseconds;

        std::string myStr;

        try {
            switch (channel) {

                case RGB:
                    //cv_const_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                    myStr = this->folder_path + std::string("/rgb_images/") + ss.str() + ".jpg";
                    break;

                case DEPTH:
                    //cv_const_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
                    myStr = this->folder_path + std::string("/depth_images/") + ss.str() + ".png";
                    break;

                default:
//		NODELET_ERROR("save_image: channel type is neither depth nor rgb, re-check your code!");
                    std::cout << "save_image: channel type is neither depth nor rgb, re-check your code!\n";
                    break;
            }
        }
        catch (cv_bridge::Exception &e) {
//	    NODELET_ERROR("cv_bridge exception: %s", e.what());
            std::cout << "cv_bridge exception: " << e.what() << "\n";
            return *this;
        }


        cv::imwrite(myStr, cv_const_ptr->image);

        return *this;
    }


    drain_base &drain_base::create_directories() {

//	ros::NodeHandle& rph = getMTPrivateNodeHandle();
        ros::NodeHandle &rph = getMyPrivateNodeHandle();

        if (!rph.getParam("base_path", base_path)) {
//	    NODELET_WARN_STREAM_NAMED("drain", "No parameter for image base path specified, will use default: "<< base_path);
            std::cout << "No parameter for image base path specified, will use default: " << base_path << "\n";
        } else {
//	    NODELET_INFO_STREAM_NAMED("drain", "Image base path: "<<base_path);
            std::cout << "Image base path: " << base_path << "\n";
        }

        std::time_t t = std::time(nullptr);
        std::tm *local_time = std::localtime(&t);


        //get the year_month_day_hour_min of this moment
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

        this->folder_path = P.string();

//	NODELET_WARN_STREAM("Folder path is set to "<<folder_path);
        std::cout << "Folder path is set to " << folder_path << "\n";

        return *this;
    }


    drain_base &drain_base::define_subscribers() {
//getMTNodeHandle allows the all publishers/subscribers to run on multiple threads in the thread pool of nodelet manager.
//	ros::NodeHandle& rh = getMTNodeHandle();
//	ros::NodeHandle& rhp = getMTPrivateNodeHandle();
        ros::NodeHandle &rh = getMyNodeHandle();
        ros::NodeHandle &rhp = getMyPrivateNodeHandle();
        
        std::cout<<"this_node::get_name= "<<ros::this_node::getName()<<"\n";

        if (!rhp.getParam("diversity", N)) {
//	    NODELET_WARN_STREAM_NAMED("camera drain", "No parameter for drain channel diversity specified, will use default value: "<< N);
            std::cout << "No parameter for drain channel diversity specified, will use default value: " << N << "\n";
        } else {
            std::cout << "Number of drain channel diversity: " << N << "\n";
        }

        depth_sub = new(std::nothrow) ros::Subscriber[N];
        rgb_sub = new(std::nothrow) ros::Subscriber[N];

        if ((!depth_sub) || (!rgb_sub)) {
//	    NODELET_FATAL("Bad memory allocation for subscribers\n");
            std::cerr << "Bad memory allocation for subscribers\n";
        }

        //define subscribers
        const std::string s_d("depth");
        const std::string s_rgb("RGB");
        for (int i = 0; i < N; i++) {
            depth_sub[i] = rh.subscribe<sensor_msgs::Image>(s_d + std::to_string(i), 8,
                                                            boost::bind(&drain_base::drain_depth_callback, this, _1,
                                                                        i));
            rgb_sub[i] = rh.subscribe<sensor_msgs::Image>(s_rgb + std::to_string(i), 8,
                                                          boost::bind(&drain_base::drain_rgb_callback, this, _1, i));
        }

/*	//experimental: let rgb_sub[0] subscribe to /camera/color/image_raw
	rgb_sub[0] = rh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 8, boost::bind(&drain::drain_rgb_callback, this, _1, 0));
	//let depth_sub[0] subscribe to /camera/aligned_depth_to_color/image_raw
	depth_sub[0] = rh.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 8, boost::bind(&drain::drain_depth_callback, this, _1, 0));
	//end experiment*/

//	NODELET_INFO_STREAM_NAMED("camera drain", "Defined "<<N<<" subscribers");
        std::cout << "Defined " << N << " subscribers\n";

        return *this;
    }
    
    void drain_base::initialize() {
        //getMTNodeHandle allows the all publishers/subscribers to run on multiple threads in the thread pool of nodelet manager.
        ros::NodeHandle &rh = getMyNodeHandle();
        ros::NodeHandle &rhp = getMyPrivateNodeHandle();
    
        define_subscribers();
        create_directories();
    
        timer = rh.createTimer(ros::Duration(5), &drain_base::timerCallback, this);
    }
    
    void drain_base::timerCallback(const ros::TimerEvent &event) {
        std::cout << "Drain: Depth received " << depth_counter.getCurrentSeq() << " frames, total loss estimate is "
                  << depth_counter.getLoss() << " frames\n";
        std::cout << "Drain: RGB received " << rgb_counter.getCurrentSeq() << " frames, total loss estimate is "
                  << rgb_counter.getLoss() << " frames\n";
    }
    
//    void drain_independent::timerCallback(const ros::TimerEvent &event) {
//        std::cout << "Drain: Depth received " << depth_counter.getCurrentSeq() << " frames, total loss estimate is "
//                  << depth_counter.getLoss() << " frames\n";
//        std::cout << "Drain: RGB received " << rgb_counter.getCurrentSeq() << " frames, total loss estimate is "
//                  << rgb_counter.getLoss() << " frames\n";
//    }
    
    
}
