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

namespace camera{

    
    void source::onInit(){
	
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
    	timer = rh.createTimer(ros::Duration(1/FPS), &source::timerCallback, this, true);
	NODELET_INFO("Camera source node onInit called\n");
    }


    void source::timerCallback(const ros::TimerEvent& event){
	boost::thread t1([&](){
		while(1){this->parallelAction();}
	    });

	boost::thread t2([&](){
		while(1){this->parallelAction();}
	    });

	boost::thread t3([&](){
		while(1){this->parallelAction();}
	    });

    }
    
	
    void source::parallelAction(){

	NODELET_DEBUG_NAMED("source", "TIMER CALLBACK CALLED");

	unsigned int second;
	unsigned int nanosecond ;

	
	sensor_msgs::Image depth_msg;
	sensor_msgs::Image rgb_msg;

	rs2::frameset frames;
	
	boost::unique_lock<boost::mutex> lock(mutex);       
	
	uint32_t channel = seq % N;
	++seq;

	while(!p.poll_for_frames(&frames));

	lock.unlock();
        //frames = p.wait_for_frames();
	double frame_timestamp =  frames.get_timestamp(); //realsense timestamp in ms
	frames = align_to_color.at(channel).process(frames);



	//we need to convert from millisecond to a [second-nanosecond] format 
	second = frame_timestamp/1000; //obtain the "second" part
	nanosecond = (frame_timestamp-1000*(double)(second))*1000000; //obtain the "nanosecond" part

	//use chrono time since epoch
	std::uint64_t nanosecond_64 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	second = unsigned(nanosecond_64/1000000000);
	nanosecond = unsigned(nanosecond_64-1000000000*(std::uint64_t)(second));
	
	rs2::depth_frame depth = frames.get_depth_frame();
       	rs2::video_frame color = frames.get_color_frame();

	unsigned int width = depth.get_width();
	unsigned int height = depth.get_height();

	unsigned int width_color = color.get_width();
	unsigned int height_color = color.get_height();
    
        uint8_t* pixel_ptr = (uint8_t*)(depth.get_data());
        uint8_t* pixel_ptr_color = (uint8_t*)(color.get_data());

	
	unsigned int pixel_amount = width*height;
	unsigned int pixel_amount_color = width_color*height_color;


	depth_msg.data = std::vector<uint8_t>(pixel_ptr, pixel_ptr + 2*pixel_amount);
	rgb_msg.data = std::vector<uint8_t>(pixel_ptr_color, pixel_ptr_color + 3*pixel_amount_color);

	//lock.unlock();

	//as for the size of the vector, since depth image is of mono16 format, one pixel corresponds to 2 bytes; RGB is of rgb8 format, where one pixel is consisted of 3 channels, 1 byte for each channel leads to a total of 3 bytes/pixel.
	//std::vector<uint8_t> depth_image(pixel_ptr, pixel_ptr + 2*pixel_amount);
	//std::vector<uint8_t> color_image(pixel_ptr_color, pixel_ptr_color + 3*pixel_amount_color);
	

	//prepare our message
        depth_msg.header.frame_id = std::to_string(seq);//this is the sequence number since start of the programme
	depth_msg.header.stamp.sec = second ;
	depth_msg.header.stamp.nsec = nanosecond;
	//depth_msg.data = depth_image;
//	depth_msg.data = std::vector<uint8_t>(pixel_ptr, pixel_ptr + 2*pixel_amount);
	depth_msg.height = height;
	depth_msg.width = width;
	depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
	depth_msg.step = width*2; //each pixel is 2 bytes, so step, which stands for row length in bytes, should be two times "width".

	rgb_msg.header.frame_id = std::to_string(seq);
	rgb_msg.header.stamp.sec = second ;
	rgb_msg.header.stamp.nsec = nanosecond ;
	//rgb_msg.data = color_image;
//	rgb_msg.data = std::vector<uint8_t>(pixel_ptr_color, pixel_ptr_color + 3*pixel_amount_color);
	rgb_msg.height = height_color;
	rgb_msg.width = width_color;
	rgb_msg.encoding = sensor_msgs::image_encodings::RGB8;
	rgb_msg.step = width_color*3;//each pixel is 3 bytes - red, green, and blue

	depth_pub[channel].publish(depth_msg);
	rgb_pub[channel].publish(rgb_msg);

	
	NODELET_DEBUG_STREAM("Both streams published to "<<channel<<"\n");
//	seq++;
    }



    source& source::define_publishers(){
	
	ros::NodeHandle& rh = getMTNodeHandle();
	ros::NodeHandle& rph = getMTPrivateNodeHandle();

	if(!rph.getParam("diversity", N)){
	    NODELET_WARN_STREAM_NAMED("camera source", "No parameter for source channel diversity specified, will use default: "<< N);
	}
	else{
	    NODELET_INFO_STREAM_NAMED("camera source", "Number of source channel diversity: "<<N);
	}

	depth_pub = new (std::nothrow) ros::Publisher[N];
	rgb_pub = new (std::nothrow) ros::Publisher[N];
		
	if((!depth_pub) || (!rgb_pub)){
	    NODELET_FATAL("Bad memory allocation for publishers\n");
	}

	//define publishers
	const std::string s_d("depth");
	const std::string s_rgb("RGB");
	for(int i=0; i<N; i++){
	    NODELET_DEBUG_STREAM_NAMED("source", "creating #" << i <<" publisher");
	    depth_pub[i] = rh.advertise<sensor_msgs::Image>(s_d+std::to_string(i), 8);
	    rgb_pub[i] = rh.advertise<sensor_msgs::Image>(s_rgb+std::to_string(i), 8);
	}

	NODELET_INFO("Publishers defined");

	return *this;
    }



    source& source::init_camera(){

	ros::NodeHandle& rph = getMTPrivateNodeHandle();
	
	if(!rph.getParam("FPS", FPS)){
	    NODELET_WARN_STREAM_NAMED("camera source", "No parameter for source FPS specified, will use default value "<<FPS);
	}
	else{
	    NODELET_INFO_STREAM_NAMED("camera source", "Source camera frame-per-second set to "<<FPS);
	}

	if(camera::Legal_FPS.find(FPS)==camera::Legal_FPS.end()){
	    NODELET_WARN_STREAM_NAMED("camera source", "Inserted illegal FPS, will use default FPS= "<< (FPS = camera::Default_FPS));
	}
	

	//configure and initialize the camera
	//according to Intel RS documentation, the max FPS of rgb stream is 60 
    	c.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, FPS);
    	c.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, (FPS>60)?60:FPS);
    	p.start(c);
	NODELET_INFO("device started!");

	return *this;
    }


    source& source::setParamTimeOfStart(){
//set a parameter that describes the time stamp of starting in microseconds
	ros::NodeHandle& rh = getMTNodeHandle();

	std::string time_of_start = helper::get_time_stamp_str();
	rh.setParam("rs_start_time", time_of_start);	
		
	NODELET_INFO_STREAM("Time-of-start set up as parameter \"start_time\", value is "<<time_of_start<<". This is used as the unique identifier for the folder created during this recording.");
	
	return *this;
    }

}
