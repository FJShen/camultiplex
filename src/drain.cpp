#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "camultiplex/helper.h"
#include "camultiplex/my_nodes.h"
#include <nodelet/nodelet.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

//include self-defined msg type
#include "camultiplex/BitLayeredImage.h"


namespace camera{

    
    
    void drain::onInit(){
	
	//getMTNodeHandle allows the all publishers/subscribers to run on multiple threads in the thread pool of nodelet manager.
	ros::NodeHandle& rh = getMTNodeHandle();
	ros::NodeHandle& rhp = getMTPrivateNodeHandle();

        if(!rhp.getParam("diversity", N)){
	    NODELET_WARN_STREAM_NAMED("camera drain", "No parameter for drain channel diversity specified, will use default value: "<< N);
	}
	else{
	    NODELET_INFO_STREAM_NAMED("camera drain", "Number of drain channel diversity: "<<N);
	}
		    
	depth_sub = new (std::nothrow) ros::Subscriber[N];
	rgb_sub = new (std::nothrow) ros::Subscriber[N];
		
	if((!depth_sub) || (!rgb_sub)){
	    NODELET_FATAL("Bad memory allocation for subscribers\n");
	}

	//define subscribers
	const std::string s_d("depth");
	const std::string s_rgb("RGB");
	for(int i=0; i<N; i++){
	    depth_sub[i] = rh.subscribe<sensor_msgs::Image>(s_d + std::to_string(i), 8, boost::bind(&drain::drain_depth_callback, this, _1, i));
	    rgb_sub[i] = rh.subscribe<sensor_msgs::Image>(s_rgb + std::to_string(i), 8, boost::bind(&drain::drain_rgb_callback, this, _1, i));
	    //depth_sub[i] = rh.subscribe<camultiplex::BitLayeredImage>(s_d + std::to_string(i), 8, boost::bind(&drain::drain_depth_callback, this, _1, i));
	    
	}

	timer = rh.createTimer(ros::Duration(12), &drain::timerCallback, this);

	NODELET_INFO("Camera drain node onInit called\n");
    }


    drain::~drain(){
	delete[] depth_sub;
	delete[] rgb_sub;
	ros::NodeHandle& rhp = getMTPrivateNodeHandle();
	rhp.deleteParam("diversity");   
	
	NODELET_INFO("camera drain node destructed\n");
    }

    
    

    //callback to handle depth frame messages
    void drain::drain_depth_callback(const sensor_msgs::Image::ConstPtr& msg, int channel_num){
	//print debug information
	NODELET_DEBUG_STREAM( "received depth frame"
			      << (msg->header.frame_id) << " from channel "
			      << std::to_string(channel_num));


	save_image(msg, DEPTH);
	depth_counter.updateSeq(std::stoul(msg->header.frame_id));
	
	//print debug information
	NODELET_DEBUG_STREAM("saved depth frame" << (msg->header.frame_id));
    }



    
    void drain::drain_rgb_callback(const sensor_msgs::Image::ConstPtr& msg, int channel_num){

        NODELET_DEBUG_STREAM( "received rgb frame"
			      << (msg->header.frame_id) << " from channel "
			      << std::to_string(channel_num));

	save_image(msg, RGB);
	rgb_counter.updateSeq(std::stoul(msg->header.frame_id));
	
        NODELET_DEBUG_STREAM("saved rgb frame" << (msg->header.frame_id));
    }




    void drain::timerCallback(const ros::TimerEvent& event){
	NODELET_WARN_STREAM("Drain: Depth received "<<depth_counter.getCurrentSeq()<<" frames, total loss estimate is "<<depth_counter.getLoss()<<" frames\n");
	NODELET_WARN_STREAM("Drain: RGB received "<<rgb_counter.getCurrentSeq()<<" frames, total loss estimate is "<<rgb_counter.getLoss()<<" frames\n");
    }


    
    
    bool drain::save_image(const sensor_msgs::Image::ConstPtr& msg, unsigned int channel){
	
	//there are two ways to convert from a Image message to openCV format
	//one is to use cv_bridge::toCvCopy, and obtain a changeable copy of the original image
	//the other way is to use cv_bridge::toCvCopy and obtain a reference to the same memory space that the ROS message holds, which forbids writing 
	cv_bridge::CvImageConstPtr cv_const_ptr;
	
	ros::NodeHandle& rh = getMTNodeHandle();
	
	std::string time_of_start;
	rh.getParam("rs_start_time", time_of_start);

	//name the image with the timestamp obtained from the camera
	//Attention! this is not time since epoch (1970) 
	unsigned int seconds = msg->header.stamp.sec;
	unsigned int nanoseconds = msg->header.stamp.nsec;
	std::stringstream ss;
	ss<<std::setw(10)<<std::setfill('0')<<seconds<<"."<<std::setw(9)<<std::setfill('0')<<nanoseconds;
	
	std::string myStr = "/media/nvidia/ExtremeSSD";
	
	//since we are merely saving the image, we do not write to the image. So we only need a reference to the original image
	try
	{
	    switch(channel){
		
	    case RGB:
		cv_const_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		myStr = myStr + "/rgb_images/" + ss.str() + ".jpg";
		break;
		
	    case DEPTH:
		cv_const_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
		myStr = myStr + "/depth_images/" + ss.str() + ".jpg";
		break;
		 
	    default:
		NODELET_ERROR("save_image: channel type is neither depth nor rgb, re-check your code!");
		break;
	    }
	}
	catch (cv_bridge::Exception& e)
	{
	    NODELET_ERROR("cv_bridge exception: %s", e.what());
	    return false;
	}
	
	
	cv::imwrite(myStr, cv_const_ptr->image);
	return true;
    }
}
