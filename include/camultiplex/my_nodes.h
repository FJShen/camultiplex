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
#include <vector>

/*
 *Define two classes: source and drain that act as our nodelets. Both inherits the base class "Nodelet::nodelet"
 *camera::source fetches new frames from the camera and sends them to the topics
 *camera::drain collects depth and rgb frames from the topics and stores them to file system
 */

#define RGB 0x0000
#define DEPTH 0x0001

namespace camera {

    const std::set<int> Legal_FPS = {15, 30, 60, 90};
    const int Default_FPS = 60;

    class source_base {
        friend class nodelet::Nodelet;

    public:
        source_base();

        virtual ~source_base();

    protected:
        ros::Publisher *depth_pub;
        ros::Publisher *rgb_pub;
        ros::Publisher T_pub;
        ros::Timer timer;

        boost::mutex seq_mutex;

        std::vector<rs2::align> align_to_color;
        std::vector<boost::thread> thread_list;

        bool enable_align = false;

        int N = 1; //this is the default number of channel diversity
        int FPS = camera::Default_FPS; //{15, 30, 60, 90}; this FPS value should be send in via command line parameters in the future
        uint32_t seq = 0;

        //rs2::frameset frames;
        rs2::pipeline p;
        rs2::config c;

        //virtual function to fetch node handle and private node handle
        virtual ros::NodeHandle &getMyNodeHandle() = 0;

        virtual ros::NodeHandle &getMyPrivateNodeHandle() = 0;

        source_base &define_publishers();

        source_base &init_camera();

        void parallelAction();

        void timerCallback(const ros::TimerEvent &event);

        source_base &setParamTimeOfStart();
        
        void initialize();
    };


    class source_nodelet : public source_base, public nodelet::Nodelet {
    protected:
        virtual ros::NodeHandle &getMyNodeHandle() override;

        virtual ros::NodeHandle &getMyPrivateNodeHandle() override;

    public:
        //onInit is an override of nodelet::Nodelet::onInit()
        virtual void onInit() override;

        virtual ~source_nodelet();
    };

    class source_independent : public source_base {
    public:
        source_independent() {
            selfInit();
        }

        virtual ~source_independent();

    private:
        ros::NodeHandle nh;
        ros::NodeHandle nph;

    protected:
        //equivalent of method void nodelet::Nodelet::onInit(), but since source_independent is not dereived from Nodelet
        //we just have to call selfInit() in constructor
        void selfInit();

        virtual ros::NodeHandle &getMyNodeHandle() override;

        virtual ros::NodeHandle &getMyPrivateNodeHandle() override;

    };


    class drain_base {
    public:

        drain_base():
        depth_sub(nullptr), rgb_sub(nullptr)
        {
//            NODELET_INFO("camera drain node constructed\n");
            std::cout << ("camera drain node base constructed\n");
        };

        virtual ~drain_base();

    protected:
        ros::Subscriber *depth_sub;
        ros::Subscriber *rgb_sub;
        ros::Timer timer;

        helper::Counter depth_counter;
        helper::Counter rgb_counter;

        std::string base_path = std::string(
                "/media/nvidia/ExtremeSSD"); //this is the path where the folder will be created
        std::string folder_path; //this is the path of the created folder

        int N = 3; //default number of channel multiplex diversity

        virtual ros::NodeHandle &getMyNodeHandle() = 0;

        virtual ros::NodeHandle &getMyPrivateNodeHandle() = 0;

        //ConstPtr& is necessary for nodelets to work
        void drain_depth_callback(const sensor_msgs::Image::ConstPtr &msg, int);

        void drain_rgb_callback(const sensor_msgs::Image::ConstPtr &msg, int);

        void timerCallback(const ros::TimerEvent &event);

        virtual drain_base &define_subscribers();

        virtual drain_base &create_directories();

        virtual drain_base &save_image(cv_bridge::CvImageConstPtr, std_msgs::Header, unsigned int);
    
        void initialize();
    };


    class drain_nodelet : public drain_base, public nodelet::Nodelet {
    protected:
        virtual ros::NodeHandle &getMyNodeHandle() override;

        virtual ros::NodeHandle &getMyPrivateNodeHandle() override;

    public:
        //drain_nodelet::onInit is override to nodelet::Nodelet::onInit
        virtual void onInit() override;

        virtual ~drain_nodelet();

    };

    class drain_independent : public drain_base {
    public:
        drain_independent() {
            selfInit();
        }

        virtual ~drain_independent();

    private:
        ros::NodeHandle nh;
        ros::NodeHandle nph; //private handle

        //equivalent of method void nodelet::Nodelet::onInit(), but since source_independent is not dereived from Nodelet
        //we just have to call selfInit() in constructor
        void selfInit();

    protected:
        virtual ros::NodeHandle &getMyNodeHandle() override;

        virtual ros::NodeHandle &getMyPrivateNodeHandle() override;

    };


}


#endif
