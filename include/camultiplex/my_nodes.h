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


#define RGB 0x0000
#define DEPTH 0x0001

/**
 * \namespace camera
 * \brief All ROS nodes for the package "camultiplex" are defined in this namespace \ref camera
 *
 * ## Purpose of design
 *
 * When using the package "camultiplex" in practice to collect images from an Intel RS435 camera and store them in any filesystem,
 * usually two nodes are involved: 1) a "source" that interfaces with the camera device, 2) a "drain" that interfaces with the file system.
 *
 * Furthermore, depending on the setup one may wish to run both nodes 1) on a single host or 2) on different hosts within a network.
 * In the case two nodes are run on the same host, it is more efficient to run them as <em>nodelets</em> which share the same memory space.
 * This eliminates the performance overhead cause by TCP communication that is ubiquitous among any two "non-nodelet" nodes.
 *
 * Based on the reasoning described above, four different classes can be defined:
 *
 * 1. source_nodelet
 * 2. drain_nodelet
 * 3. source_independent (non-nodelet)
 * 4. drain_independent (non-nodelet)
 *
 *   It would turn out that for the drain and the source side, respectively, the difference between codes for the nodelet version
 *   and the independent version is very small but subtle. In order to promote code re-usability,
 *   two base classes that serve as common "anscestors" should be added:
 *
 * 5. source_base
 * 6. drain_base
 *
 * ## Methodology of design
 *
 * The subtle difference between a nodelet and a non-nodelet is merely how they obtain their node handles.
 * The base classes (source_base and drain_base) will implement most functionality but leave the methods that returns
 * their (public) node handle and private node handle as pure virtual functions to be defined.
 *
 * Refer to external source for how node handles work: http://wiki.ros.org/roscpp/Overview/NodeHandles, http://wiki.ros.org/nodelet
 *
 * //todo: can a nodelet and a non-nodelet talk with each other?
 *
 * \see source_base.cpp, source_independent.cpp, source_nodelet.cpp
 * \see drain_base.cpp, drain_independent.cpp, drain_nodelet.cpp
 */

namespace camera {
    
    const std::set<int> Legal_FPS = {15, 30, 60, 90};
    const int Default_FPS = 60;
    
    class source_base {
        friend class nodelet::Nodelet;
    
    public:
        source_base();
        
        virtual ~source_base();
    
    protected:
        ros::Publisher* depth_pub;
        ros::Publisher* rgb_pub;
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
        virtual ros::NodeHandle& getMyNodeHandle() = 0;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() = 0;
        
        source_base& define_publishers();
        
        source_base& init_camera();
        
        void parallelAction();
        
        void timerCallback(const ros::TimerEvent& event);
        
        source_base& setParamTimeOfStart();
        
        void initialize();
    };
    
    
    class source_nodelet : public source_base, public nodelet::Nodelet {
    protected:
        virtual ros::NodeHandle& getMyNodeHandle() override;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
    
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
        
        virtual ros::NodeHandle& getMyNodeHandle() override;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
        
    };
    
    
    class drain_base {
    public:
        
        drain_base() :
                depth_sub(nullptr), rgb_sub(nullptr) {
//            NODELET_INFO("camera drain node constructed\n");
            std::cout << ("camera drain node base constructed\n");
        };
        
        virtual ~drain_base();
    
    protected:
        ros::Subscriber* depth_sub;
        ros::Subscriber* rgb_sub;
        ros::Timer timer;
        
        helper::Counter depth_counter;
        helper::Counter rgb_counter;
        
        std::string base_path = std::string(
                "/media/nvidia/ExtremeSSD"); //this is the path where the folder will be created
        std::string folder_path; //this is the path of the created folder
        
        int N = 3; //default number of channel multiplex diversity
        
        virtual ros::NodeHandle& getMyNodeHandle() = 0;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() = 0;
        
        //ConstPtr& is necessary for nodelets to work
        void drain_depth_callback(const sensor_msgs::Image::ConstPtr& msg, int);
        
        void drain_rgb_callback(const sensor_msgs::Image::ConstPtr& msg, int);
        
        void timerCallback(const ros::TimerEvent& event);
        
        virtual drain_base& define_subscribers();
        
        virtual drain_base& create_directories();
        
        virtual drain_base& save_image(cv_bridge::CvImageConstPtr, std_msgs::Header, unsigned int);
        
        void initialize();
    };
    
    
    class drain_nodelet : public drain_base, public nodelet::Nodelet {
    protected:
        virtual ros::NodeHandle& getMyNodeHandle() override;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
    
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
        virtual ros::NodeHandle& getMyNodeHandle() override;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
        
    };
    
    
}


#endif
