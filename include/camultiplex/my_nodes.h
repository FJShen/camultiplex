#ifndef CAMULTIPLEX__MY_NODES
#define CAMULTIPLEX__MY_NODES

#include "ros/ros.h"
#include <librealsense2/rs.hpp>
#include "camultiplex/helper.h"
#include <nodelet/nodelet.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <stdio.h>
#include "cv_bridge/cv_bridge.h"
#include <boost/thread.hpp>

/*
 *Define two classes: source and drain that act as our nodelets. Both inherits the base class "Nodelet::nodelet"
 *camera::source fetches new frames from the camera and sends them to the topics
 *camera::drain collects depth and rgb frames from the topics and stores them to file system
 */

#define RGB 0x0000
#define DEPTH 0x0001

namespace camera{

const std::set<int> Legal_FPS = {15, 30, 60, 90};
const int Default_FPS = 60;

    class source : public nodelet::Nodelet{
    public:
	
	source():align_to_color(RS2_STREAM_COLOR)
	{
	    NODELET_INFO("camera source node constructed\n");
	};
	
	~source(){
	    delete[] depth_pub;
	    delete[] rgb_pub;
	    
	    ros::NodeHandle& rhp = getMTPrivateNodeHandle();
	    rhp.deleteParam("diversity");
	    rhp.deleteParam("FPS");
	    
	    ros::NodeHandle& rh = getMTNodeHandle();
	    rh.deleteParam("rs_start_time");
	    
	    p.stop();
	    
	    NODELET_INFO("camera source node destrcuted\n");
	};
	
	virtual void onInit();//mandatory initialization function for all nodelets

    private:
	ros::Publisher* depth_pub;
	ros::Publisher* rgb_pub;
	ros::Publisher T_pub;
	ros::Timer timer;

	int N = 2; //this is the default number of channel diversity 
	int FPS = 60; //{15, 30, 60, 90}; this FPS value should be send in via command line parameters in the future
	uint32_t seq = 0;
	
	//rs2::frameset frames;
	rs2::pipeline p;
	rs2::config c;
	rs2::align align_to_color;
	rs2::frame_queue fq;
	
	//callback function that transmits frames to the topics
	void timerCallback(const ros::TimerEvent& event);

	source& define_publishers();
	source& init_camera();
	source& setParamTimeOfStart();
    };


 
    class drain : public nodelet::Nodelet{
    public:
	
	drain(){  
	    NODELET_INFO("camera drain node constructed\n");
	};
	
	~drain();
	
	virtual void onInit();//mandatory initialization function for all nodelets
	
    private:
	ros::Subscriber* depth_sub;
	ros::Subscriber* rgb_sub;
	ros::Timer timer;
	
	helper::counter depth_counter;
	helper::counter rgb_counter;

	std::string base_path = std::string("/media/nvidia/ExtremeSSD"); //this is the path where the folder will be created
	std::string folder_path; //this is the path of the created folder

	
       
	int N=3; //default number of channel multiplex diversity

	//ConstPtr& is necessary for nodelets to work
	void drain_depth_callback(const sensor_msgs::Image::ConstPtr& msg, int);
	void drain_rgb_callback(const sensor_msgs::Image::ConstPtr& msg, int);
	void timerCallback(const ros::TimerEvent& event);

	drain& define_subscribers();
	drain& create_directories();
	drain& save_image(cv_bridge::CvImageConstPtr, std_msgs::Header, unsigned int);
    };


   
}


#endif
