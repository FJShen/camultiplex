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

        source_base()
        {
            std::cout << "camera source base node constructed\n";
        };

        virtual ~source_base() {
            for (auto &x : thread_list) {
                if (x.get_id() != boost::thread::id()) {
                    x.interrupt();
                }
            }
    
            for (auto &x : thread_list) {
                if (!x.try_join_for(boost::chrono::milliseconds(100))) {
                    std::cerr << ("failed to join a thread\n");
                }
            }
    
            p.stop();
            delete[] depth_pub;
            delete[] rgb_pub;
            std::cout << ("camera source base node destrcuted\n");
        };

        //virtual void onInit();//mandatory initialization function for all nodelets

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

        virtual source_base &define_publishers();

        virtual source_base &init_camera();

        //This is the kernel of operation; the conversion from rs2::frameset to sensor_msgs::Image is done here
        virtual void parallelAction();

        virtual void timerCallback(const ros::TimerEvent &event) = 0;

        source_base &setParamTimeOfStart();
        
        void initialize();
    };


    class source_nodelet : public source_base, public nodelet::Nodelet {
    protected:
        virtual ros::NodeHandle &getMyNodeHandle() override {
            return getMTNodeHandle();
        }

        virtual ros::NodeHandle &getMyPrivateNodeHandle() override {
            return getMTPrivateNodeHandle();
        }

        //callback function that transmits frames to the topics
        virtual void timerCallback(const ros::TimerEvent &event) override;

    public:
        virtual void onInit() override {
            initialize();
            std::cout << ("Camera source node nodelet onInit called\n");
        }

        virtual ~source_nodelet() {
            ros::NodeHandle &rhp = getMyPrivateNodeHandle();
            rhp.deleteParam("diversity");
            rhp.deleteParam("FPS");
            rhp.deleteParam("align");
            
            ros::NodeHandle &rh = getMyNodeHandle();
            rh.deleteParam("rs_start_time");
        }
    };

    class source_independent : public source_base {
    public:
        source_independent() : nph("source") {
            selfInit();
        }

        virtual ~source_independent() {
            ros::NodeHandle &rhp = getMyPrivateNodeHandle();
            rhp.deleteParam("diversity");
            rhp.deleteParam("FPS");
            rhp.deleteParam("align");

            ros::NodeHandle &rh = getMyNodeHandle();
            rh.deleteParam("rs_start_time");
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle nph;

    protected:
        //equivalent of method void nodelet::Nodelet::onInit(), but since source_independent is not dereived from Nodelet
        //we just have to call selfInit() in constructor
        void selfInit() {
            initialize();
            std::cout << ("Camera independent source node onInit called\n");
        }

        virtual ros::NodeHandle &getMyNodeHandle() override {
            return nh;
        }

        virtual ros::NodeHandle &getMyPrivateNodeHandle() override {
            return nph;
        }

        virtual void timerCallback(const ros::TimerEvent &event) override;

    };


    class drain_base {
    public:

        drain_base() {
//            NODELET_INFO("camera drain node constructed\n");
            std::cout << ("camera drain node base constructed\n");
        };

        virtual ~drain_base();

    protected:
        ros::Subscriber *depth_sub;
        ros::Subscriber *rgb_sub;
        ros::Timer timer;

        helper::counter depth_counter;
        helper::counter rgb_counter;

        std::string base_path = std::string(
                "/media/nvidia/ExtremeSSD"); //this is the path where the folder will be created
        std::string folder_path; //this is the path of the created folder

        int N = 3; //default number of channel multiplex diversity

        virtual ros::NodeHandle &getMyNodeHandle() = 0;

        virtual ros::NodeHandle &getMyPrivateNodeHandle() = 0;

        //ConstPtr& is necessary for nodelets to work
        void drain_depth_callback(const sensor_msgs::Image::ConstPtr &msg, int);

        void drain_rgb_callback(const sensor_msgs::Image::ConstPtr &msg, int);

        virtual void timerCallback(const ros::TimerEvent &event) = 0;

        virtual drain_base &define_subscribers();

        virtual drain_base &create_directories();

        virtual drain_base &save_image(cv_bridge::CvImageConstPtr, std_msgs::Header, unsigned int);
    };


    class drain_nodelet : public drain_base, public nodelet::Nodelet {
    private:
        virtual ros::NodeHandle &getMyNodeHandle() override {
            return getMTNodeHandle();
        }

        virtual ros::NodeHandle &getMyPrivateNodeHandle() override {
            return getMTPrivateNodeHandle();
        }

        virtual void timerCallback(const ros::TimerEvent &event) override;

    public:
        //drain_nodelet::onInit is override to nodelet::Nodelet::onInit
        virtual void onInit() override {

            //getMTNodeHandle allows the all publishers/subscribers to run on multiple threads in the thread pool of nodelet manager.
            ros::NodeHandle &rh = getMyNodeHandle();
            ros::NodeHandle &rhp = getMyPrivateNodeHandle();

            define_subscribers();
            create_directories();

            timer = rh.createTimer(ros::Duration(5), &drain_nodelet::timerCallback, this);

            NODELET_INFO("Camera drain node onInit called\n");
        }

        virtual ~drain_nodelet() {
            delete[] depth_sub;
            delete[] rgb_sub;
            ros::NodeHandle &rhp = getMyPrivateNodeHandle();
            rhp.deleteParam("diversity");
            rhp.deleteParam("base_path");
        }

    };

    class drain_independent : public drain_base{
    public:
        drain_independent() : nph("drain"){
            selfInit();
        }

        virtual ~drain_independent() {
            delete[] depth_sub;
            delete[] rgb_sub;
            ros::NodeHandle &rhp = getMyPrivateNodeHandle();
            rhp.deleteParam("diversity");
            rhp.deleteParam("base_path");
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle nph; //private handle

        //equivalent of method void nodelet::Nodelet::onInit(), but since source_independent is not dereived from Nodelet
        //we just have to call selfInit() in constructor
        void selfInit(){

            //getMTNodeHandle allows the all publishers/subscribers to run on multiple threads in the thread pool of nodelet manager.
            ros::NodeHandle &rh = getMyNodeHandle();
            ros::NodeHandle &rhp = getMyPrivateNodeHandle();

            define_subscribers();
            create_directories();

            timer = rh.createTimer(ros::Duration(5), &drain_independent::timerCallback, this);

            std::cout<<("Camera independent drain node selfInit called\n");
        }

    protected:
        virtual ros::NodeHandle &getMyNodeHandle() override{
            return nh;
        }

        virtual ros::NodeHandle &getMyPrivateNodeHandle() override{
            return nph;
        }

        virtual void timerCallback(const ros::TimerEvent &event) override;
    };


}


#endif
