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
#include <opencv2/opencv.hpp>

/*
 *Define two classes: source and drain that act as our nodelets. Both inherits the base class "Nodelet::nodelet"
 *camera::source fetches new frames from the camera and sends them to the topics
 *camera::drain collects depth and rgb frames from the topics and stores them to file system
 */

#define RGB 0x0000
#define DEPTH 0x0001

namespace camera{

    class source : public nodelet::Nodelet{
    public:
	
	source(){
	    NODELET_INFO("camera source node constructed\n");
	};
	
	~source(){
	    delete[] depth_pub;
	    delete[] rgb_pub;
	    ros::NodeHandle& rhp = getMTPrivateNodeHandle();
	    rhp.deleteParam("diversity");
	    ros::NodeHandle& rh = getMTNodeHandle();
	    rh.deleteParam("rs_start_time");
	    NODELET_INFO("camera source node destrcuted\n");
	};
	
	virtual void onInit();//mandatory initialization function for all nodelets

    private:
	ros::Publisher* depth_pub;
	ros::Publisher* rgb_pub;
	ros::Timer timer;

	int N = 2; //this is the default number of channel diversity 
	float FPS = 60; //{15, 30, 60, 90}; this FPS value should be send in via command line parameters in the future
	uint32_t seq = 0;
	
	//rs2::frameset frames;
	rs2::pipeline p;
	rs2::config c;
	
	//callback function that transmits frames to the topics
	void timerCallback(const ros::TimerEvent& event);
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
       
	int N=3; //default number of channel multiplex diversity

	//ConstPtr& is necessary for nodelets to work
	void drain_depth_callback(const sensor_msgs::Image::ConstPtr& msg, int);
	void drain_rgb_callback(const sensor_msgs::Image::ConstPtr& msg, int);

	void timerCallback(const ros::TimerEvent& event);
	
	bool save_image(const sensor_msgs::Image::ConstPtr&, unsigned int);
    };


   
}


#endif
