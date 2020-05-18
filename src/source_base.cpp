#include "ros/ros.h"
#include <librealsense2/rs.hpp>
#include "camultiplex/helper.h"
#include "camultiplex/my_nodes.h"
#include <nodelet/nodelet.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <boost/filesystem.hpp>

#include <boost/thread.hpp>
#include <vector>


namespace camera {


/*    void source_base::onInit(){
	
	//getMTNodeHandle allows the all publishers/subscribers to run on multiple threads in the thread pool of nodelet manager.
	ros::NodeHandle& rh = getMTNodeHandle();
	ros::NodeHandle& rph = getMTPrivateNodeHandle();

	this->define_publishers()
	    .init_camera()
	    .setParamTimeOfStart();

	for(int i=0; i<N; i++){
	    align_to_color.emplace_back(RS2_STREAM_COLOR);
	}

	//use timer to trigger callback

  timer = rh.createTimer(ros::Duration(1/FPS), &source_base::timerCallback, this, true);

	std::cout<<("Camera source node onInit called\n");
    }
*/

    void source_nodelet::timerCallback(const ros::TimerEvent &event) {

        //define lambda which will be launched in parallel
        auto f = [&]() {
            try {
                while (1) { this->parallelAction(); }
            }
            catch (boost::thread_interrupted &) {
                return;
            }
        };

        for (int i = 0; i < N; i++) {
            thread_list.emplace_back(f);
        }
    }

    void source_independent::timerCallback(const ros::TimerEvent &event) {

        //define lambda which will be launched in parallel
        auto f = [&]() {
            try {
                while (1) { this->parallelAction(); }
            }
            catch (boost::thread_interrupted &) {
                return;
            }
        };

        for (int i = 0; i < N; i++) {
            thread_list.emplace_back(f);
        }
    }


    void source_base::parallelAction() {

        boost::this_thread::interruption_point();

        //NODELET_DEBUG_NAMED("source", "TIMER CALLBACK CALLED");
        //std::cout << "TIMER CALLBACK CALLED\n";

        unsigned int second;
        unsigned int nanosecond;


        sensor_msgs::Image depth_msg;
        sensor_msgs::Image rgb_msg;

        rs2::frameset frames;
    
        boost::unique_lock<boost::mutex> seq_lock(seq_mutex);
        
        frames = p.wait_for_frames();

        uint32_t local_seq = seq;
        ++seq;

        seq_lock.unlock();

        if (enable_align) {
            frames = align_to_color.at(local_seq % N).process(frames);
        }
    
    
        double frame_timestamp = frames.get_timestamp(); //realsense timestamp in ms
        //we need to convert from millisecond to a [second-nanosecond] format
        second = frame_timestamp / 1000; //obtain the "second" part
        nanosecond = (frame_timestamp - 1000 * (double) (second)) * 1000000; //obtain the "nanosecond" part. Need to convert "second" to double otherwise multiplying it by 1000 will likely overflow uint's range

//        //use chrono time since epoch
//        std::uint64_t nanosecond_64 = std::chrono::duration_cast<std::chrono::nanoseconds>(
//                std::chrono::system_clock::now().time_since_epoch()).count();
//        second = unsigned(nanosecond_64 / 1000000000);
//        nanosecond = unsigned(nanosecond_64 - 1000000000 * (std::uint64_t) (second));

        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::video_frame color = frames.get_color_frame();

        unsigned int width = depth.get_width();
        unsigned int height = depth.get_height();

        unsigned int width_color = color.get_width();
        unsigned int height_color = color.get_height();

        uint8_t *pixel_ptr = (uint8_t *) (depth.get_data());
        uint8_t *pixel_ptr_color = (uint8_t *) (color.get_data());


        unsigned int pixel_amount = width * height;
        unsigned int pixel_amount_color = width_color * height_color;

        //As for the size of the vector, since depth image is of mono16 format, one pixel corresponds to 2 bytes; RGB is of rgb8 format, where one pixel is consisted of 3 channels, 1 byte for each channel leads to a total of 3 bytes/pixel.
        //The type of sensor_msgs::Image.data is std::vector<uint8_t>
        //Reference: https://docs.ros.org/diamondback/api/sensor_msgs/html/Image_8h_source.html
        depth_msg.data = std::vector<uint8_t>(pixel_ptr, pixel_ptr + 2 * pixel_amount);
        rgb_msg.data = std::vector<uint8_t>(pixel_ptr_color, pixel_ptr_color + 3 * pixel_amount_color);

        //prepare rest of the part of our message
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

        depth_pub[local_seq % N].publish(depth_msg);
        rgb_pub[local_seq % N].publish(rgb_msg);


        //NODELET_DEBUG_STREAM("Both streams published to " << local_seq % N << "\n");
        //std::cout << "Both streams published to " << local_seq % N << "\n";
    }


    source_base &source_base::define_publishers() {

//        ros::NodeHandle &rh = getMTNodeHandle();
//        ros::NodeHandle &rph = getMTPrivateNodeHandle();
        ros::NodeHandle &rh = getMyNodeHandle();
        ros::NodeHandle &rph = getMyPrivateNodeHandle();

        if (!rph.getParam("diversity", N)) {
//            NODELET_WARN_STREAM_NAMED("camera source",
//                                      "No parameter for source channel diversity specified, will use default: " << N);
            std::cout << "No parameter for source channel diversity specified, will use default: " << N<<"\n";
        } else {
//            NODELET_INFO_STREAM_NAMED("camera source", "Number of source channel diversity: " << N);
            std::cout << "Number of source channel diversity: " << N<<"\n";
        }

        depth_pub = new(std::nothrow) ros::Publisher[N];
        rgb_pub = new(std::nothrow) ros::Publisher[N];

        if ((!depth_pub) || (!rgb_pub)) {
//            NODELET_FATAL("Bad memory allocation for publishers\n");
            std::cerr << ("Bad memory allocation for publishers\n");
        }

        //define publishers
        const std::string s_d("depth");
        const std::string s_rgb("RGB");
        for (int i = 0; i < N; i++) {
//            NODELET_DEBUG_STREAM_NAMED("source", "creating #" << i << " publisher");
            std::cout << "creating #" << i << " publisher\n";
            depth_pub[i] = rh.advertise<sensor_msgs::Image>(s_d + std::to_string(i), 8);
            rgb_pub[i] = rh.advertise<sensor_msgs::Image>(s_rgb + std::to_string(i), 8);
        }

//        NODELET_INFO("Publishers defined");
        std::cout << ("Publishers defined\n");

        return *this;
    }


    source_base &source_base::init_camera() {

//        ros::NodeHandle &rph = getMTPrivateNodeHandle();
        ros::NodeHandle &rph = getMyPrivateNodeHandle();

        //get align param
        if (!rph.getParam("align", enable_align)) {
//            NODELET_WARN_STREAM_NAMED("camera source",
//                                      "No parameter for alignment on/off specified, will use default value "
//                                              << (enable_align ? "on" : "off"));
            std::cout << "No parameter for alignment on/off specified, will use default value "
                      << (enable_align ? "on\n" : "off\n");
        } else {
//            NODELET_WARN_STREAM_NAMED("camera source",
//                                      "Depth alignment is turned on? " << (enable_align ? "yes" : "no"));
            std::cout << "Depth alignment is turned on? " << (enable_align ? "yes\n" : "no\n");
        }
    
        //get FPS param
        if (!rph.getParam("FPS", FPS)) {
//            NODELET_WARN_STREAM_NAMED("camera source",
//                                      "No parameter for source FPS specified, will use default value " << FPS);
            std::cout << "No parameter for source FPS specified, will use default value " << FPS<<"\n";
        } else {
//            NODELET_INFO_STREAM_NAMED("camera source", "Source camera frame-per-second set to " << FPS);
            std::cout << "Source camera frame-per-second set to " << FPS<<"\n";
        }
        
        //check FPS validity
        if (camera::Legal_FPS.find(FPS) == camera::Legal_FPS.end()) {
            //std::set<T>.find(xxx) returns std::set<T>.end() if xxx is not found
//            NODELET_WARN_STREAM_NAMED("camera source",
//                                      "Inserted illegal FPS, will use default FPS= " << (FPS = camera::Default_FPS));
            std::cout << "FPS value is illegal, will use default FPS= " << (FPS = camera::Default_FPS)<<"\n";
        }
        
        if(FPS>60){
            std::cout<<"Warning: Per Intel RS 435 documentation, the maximum supported FPS for RGB stream is 60.\n";
        }


        //configure and initialize the camera
        //according to Intel RS documentation, the max FPS of rgb stream is 60
        c.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, FPS);
        c.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, (FPS > 60) ? 60 : FPS);
        try {
            p.start(c);
        }
        catch(rs2::error& err){
            std::cerr<<"Caught rs2::error, failed_function="<<err.get_failed_function()<<", message= "<<err.what()<<"\n";
        }
//        NODELET_INFO("device started!");
        std::cout << ("device started!\n");

        return *this;
    }


    source_base &source_base::setParamTimeOfStart() {
//set a parameter that describes the time stamp of starting in microseconds
//        ros::NodeHandle &rh = getMTNodeHandle();
        ros::NodeHandle &rh = getMyNodeHandle();

        std::string time_of_start = helper::get_time_stamp_str();
        rh.setParam("rs_start_time", time_of_start);

//        NODELET_INFO_STREAM("Time-of-start set up as parameter \"start_time\", value is " << time_of_start
//                                                                                          << ". This is used as the unique identifier for the folder created during this recording.");
        std::cout << "Time-of-start set up as parameter \"start_time\", value is " << time_of_start
                  << ". This is used as the unique identifier for the folder created during this recording.\n";

        return *this;
    }
    
    void source_base::initialize(){
        //getMTNodeHandle allows the all publishers/subscribers to run on multiple threads in the thread pool of nodelet manager.
        ros::NodeHandle &rh = getMyNodeHandle();
        ros::NodeHandle &rph = getMyPrivateNodeHandle();
    
        //tentatively moved to source_base ctor
        define_publishers();
        init_camera();
        setParamTimeOfStart();
    
        for(int i=0; i<N; ++i){
            align_to_color.emplace_back(RS2_STREAM_COLOR);
        }
    
        //use timer to trigger callback
    
        timer = rh.createTimer(ros::Duration(1 / FPS), &source_base::timerCallback, this, true);
    }

}
