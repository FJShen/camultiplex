#include "ros/ros.h"
#include <librealsense2/rs.hpp>
#include "camultiplex/helper.h"
#include "camultiplex/my_nodes.h"
#include <nodelet/nodelet.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include <boost/thread.hpp>
#include <vector>


namespace camera {
    
    Source_base::Source_base() :
            depth_pub(nullptr), rgb_pub(nullptr) {
        //std::cout << "camera Source_base node constructed\n";
    }
    
    
    Source_base::~Source_base() {
        for (auto& x : thread_list) {
            if (x.get_id() != boost::thread::id()) {
                x.interrupt();
            }
        }
        
        for (auto& x : thread_list) {
            if (!x.try_join_for(boost::chrono::milliseconds(100))) {
                std::cout << ("failed to join a thread\n");
            }
        }
        
        p.stop();
        delete[] depth_pub;
        delete[] rgb_pub;
        std::cout << ("camera source base node destrcuted\n");
    };
    
    
    void Source_base::initialize() {
        
        std::cout << "Initializing source...\n";
    
        initCamera();
        definePublishers();
        setParamTimeOfStart();
        
        for (int i = 0; i < diversity; ++i) {
            align_to_color.emplace_back(RS2_STREAM_COLOR);
        }
        
        launchThreads();
        
        std::cout << "Source initialized!\n";
    }
    
    
    Source_base& Source_base::definePublishers() {
        
        ros::NodeHandle& private_handle = getMyPrivateNodeHandle();
        ros::NodeHandle& handle = getMyNodeHandle();
        
        if (!private_handle.getParam("diversity", diversity)) {
            std::cout << "No parameter for source channel diversity specified, will use default: " << diversity << "\n";
        } else {
            std::cout << "Number of source channel diversity: " << diversity << "\n";
        }
        
        depth_pub = new ros::Publisher[diversity];
        rgb_pub = new ros::Publisher[diversity];
        
        //define publishers
        const std::string s_d("depth");
        const std::string s_rgb("RGB");
        for (int i = 0; i < diversity; i++) {
            std::cout << "creating publisher #" << i << "\n";
            depth_pub[i] = handle.advertise<sensor_msgs::Image>(s_d + std::to_string(i), 8);
            rgb_pub[i] = handle.advertise<sensor_msgs::Image>(s_rgb + std::to_string(i), 8);
        }
        
        std::cout << ("Publishers defined\n");
        return *this;
    }
    
    
    Source_base& Source_base::initCamera() {
        
        ros::NodeHandle& private_handle = getMyPrivateNodeHandle();
        
        //get align param
        if (!private_handle.getParam("align", enable_align)) {
            std::cout << "No parameter for alignment on/off specified, will use default value "
                      << (enable_align ? "ON\n" : "OFF\n");
        } else {
            std::cout << "Depth alignment is turned " << (enable_align ? "ON\n" : "OFF\n");
        }
        
        //get FPS param
        if (!private_handle.getParam("FPS", FPS)) {
            std::cout << "No parameter for source FPS specified, will use default value " << FPS << "\n";
        } else {
            std::cout << "Source camera frame-per-second set to " << FPS << "\n";
        }
        
        //check FPS validity
        if (camera::Legal_FPS.find(FPS) == camera::Legal_FPS.end()) {
            //std::set<T>.find(xxx) returns std::set<T>.end() if xxx is not found
            std::cout << "FPS value is illegal, will use default FPS = " << (FPS = camera::Default_FPS) << "\n";
        }
        
        if (FPS > 60) {
            std::cout << "Warning: Per Intel RS 435 documentation, the maximum supported FPS for RGB stream is 60. Source will automatically capture RGB frames at 60 Hz\n";
        }
        
        
        //configure and initialize the camera
        //according to Intel RS documentation, the max FPS of rgb stream is 60
        c.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, FPS);
        c.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, (FPS > 60) ? 60 : FPS);
        
        //let the camera start working
        try {
            p.start(c);
        }
        catch (rs2::error& err) {
            std::cout << "Caught rs2::error, failed_function=" << err.get_failed_function() << ", message= "
                      << err.what() << "\n";
        }
        
        std::cout << ("Device started\n");
        
        return *this;
    }
    
    
    Source_base& Source_base::setParamTimeOfStart() {
    //set a parameter that describes the time stamp of starting in microseconds
        ros::NodeHandle& handle = getMyNodeHandle();
        
        std::string time_of_start = helper::get_time_stamp_str();
        handle.setParam("rs_start_time", time_of_start);
        
        std::cout << "Time-of-start set up as parameter \"rs_start_time\", value is " << time_of_start
                  << "\n";
        
        return *this;
    }
    
    
    void Source_base::kernelRoutine() {
        
        //this interruption point is where the thread will exit when the destructor wants to kill a thread that is running this routine
        boost::this_thread::interruption_point();
        
        unsigned int second; //"second" part of Realsense timestamp
        unsigned int nanosecond; //"nanosecond" part of Realsense timestamp
        double frame_timestamp; //the original Realsense timestamp
        
        sensor_msgs::Image depth_msg;
        sensor_msgs::Image rgb_msg;
        
        rs2::frameset frames;
        
        //Use lock to ensure multi-thread safety
        boost::unique_lock<boost::mutex> seq_lock(seq_mutex);
        
        frames = p.wait_for_frames();
        uint32_t local_seq = seq;
        ++seq;
        
        //after obtaining a frame and updating ther serial number, no need to keep the mutex locked
        seq_lock.unlock();
        
        //do depth alignment if required
        if (enable_align) {
            frames = align_to_color.at(local_seq % diversity).process(frames);
        }
        
        frame_timestamp = frames.get_timestamp(); //realsense timestamp in ms
        
        //we need to convert from millisecond to a [second-nanosecond] format
        second = frame_timestamp / 1000;
        nanosecond = (frame_timestamp - 1000 * (double) (second)) * 1000000; //obtain the "nanosecond" part. Need to convert "second" to double otherwise multiplying it by 1000 will likely overflow uint's range
        
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::video_frame color = frames.get_color_frame();
        
        unsigned int width = depth.get_width();
        unsigned int height = depth.get_height();
        
        unsigned int width_color = color.get_width();
        unsigned int height_color = color.get_height();
    
        unsigned int pixel_amount = width * height;
        unsigned int pixel_amount_color = width_color * height_color;
        
        uint8_t* pixel_ptr = (uint8_t*) (depth.get_data());
        uint8_t* pixel_ptr_color = (uint8_t*) (color.get_data());
        
        
        //As for the size of the vector, since depth image is of mono16 format, one pixel corresponds to 2 bytes;
        //RGB is of rgb8 format, where one pixel is consisted of 3 channels, 1 byte for each channel leads to a total of 3 bytes/pixel.
        //The type of sensor_msgs::Image.data is std::vector<uint8_t>
        //Reference: https://docs.ros.org/diamondback/api/sensor_msgs/html/Image_8h_source.html
        depth_msg.data = std::vector<uint8_t>(pixel_ptr, pixel_ptr + 2 * pixel_amount);
        rgb_msg.data = std::vector<uint8_t>(pixel_ptr_color, pixel_ptr_color + 3 * pixel_amount_color);
        
        //prepare rest of the part of sensor_msgs::Image
        depth_msg.header.frame_id = std::to_string(local_seq);//this is the sequence number since start of the programme
        depth_msg.header.stamp.sec = second;
        depth_msg.header.stamp.nsec = nanosecond;
        depth_msg.height = height;
        depth_msg.width = width;
        depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
        depth_msg.step = width * 2; //each pixel is 2 bytes; so step, which stands for row length in bytes, should be two times "width".
        
        rgb_msg.header.frame_id = std::to_string(local_seq);
        rgb_msg.header.stamp.sec = second;
        rgb_msg.header.stamp.nsec = nanosecond;
        rgb_msg.height = height_color;
        rgb_msg.width = width_color;
        rgb_msg.encoding = sensor_msgs::image_encodings::RGB8;
        rgb_msg.step = width_color * 3;//each pixel is 3 bytes - red, green, and blue
        
        //publish to one of the publishers
        //TODO: can we strictly allocate a specific publisher to each thread in Source_nodelet::launchThreads?
        depth_pub[local_seq % diversity].publish(depth_msg);
        rgb_pub[local_seq % diversity].publish(rgb_msg);
    }
    
    
    void Source_base::launchThreads() {
        //define lambda which will be launched in parallel
        auto infinite_loop = [&]() {
            try {
                while (1) this->kernelRoutine();
            }
    
            catch (boost::thread_interrupted&) {
                return;
            }
        };
        
        for (int i = 0; i < diversity; i++) {
            thread_list.emplace_back(infinite_loop);
        }
    }
    
}
