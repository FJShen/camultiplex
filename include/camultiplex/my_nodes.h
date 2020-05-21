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
 * \brief All ROS nodes for the package "camultiplex" are defined in this namespace
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
 * 1. Source_nodelet
 * 2. Drain_nodelet
 * 3. Source_independent (non-nodelet)
 * 4. Drain_independent (non-nodelet)
 *
 *   It would turn out that for the drain and the source side, respectively, the difference between codes for the nodelet version
 *   and the independent version is very small but subtle. In order to promote code re-usability,
 *   two base classes that serve as common "anscestors" should be added:
 *
 * 5. Source_base
 * 6. Drain_base
 *
 * ## Methodology of design
 *
 * The subtle difference between a nodelet and a non-nodelet is merely how they obtain their node handles.
 * The base classes (Source_base and Drain_base) will implement most essential functionality but leave the methods that returns
 * their (public) node handle and private node handle as pure virtual functions to be defined. Source_base and Drain_base, therefore,
 * are virtual base classes that cannot be instantiated.
 *
 * Refer to external source for how node handles work: http://wiki.ros.org/roscpp/Overview/NodeHandles, http://wiki.ros.org/nodelet
 *
 * //todo: can a nodelet and a non-nodelet talk with each other?
 *
 * \see Source_base, Drain_base
 */
namespace camera {
    
    const std::set<int> Legal_FPS = {15, 30, 60, 90}; ///< Acceptable FPS values supported by Intel RS435.
    const int Default_FPS = 60; ///< Default FPS value when no value was provided to rosrun or roslaunch
    
    
    /**
     * \class Source_base
     * \brief Interfaces with the camera and publishes the image frames in the form of sensor_msgs::Image
     *
     * ## Configurable parameters
     * 1. diversity
     * 2. FPS
     * 3. align
     *
     * ## Usage
     * Source_base is an abstract class that provides nearly all functionality needed by a source node which interfaces with the camera.
     * Its two pure virtual functions, \ref getMyNodeHandle and \ref getMyPrivateNodeHandle need to be implemented by derived class depending on whether they are nodes or nodelets.
     */
    class Source_base {
    
    public:
        Source_base(); ///<Constructor
        
        virtual ~Source_base(); ///<Destructor
    
    protected:
        
        virtual ros::NodeHandle& getMyNodeHandle() = 0; ///<virtual function to fetch node handle
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() = 0; ///<virtual function to fetch private node handle
        
        
        /**
         * \brief Initialize and start the source node.
         *
         * This method obtains the parameters (FPS, align, diversity) from ROS parameter server and does the following in order:
         * 1. Start the camera with the required FPS and alignment setting
         * 2. Populate rgb_pub and depth_pub with the required amount of publishers
         * 3. Set up a new parameter called "rs_start_time". The drain node will have access to this parameter, though it is currently not utilized.
         * 4. Instantiate matching number of alignment processing blocks and put them into the vector \ref align_to_color
         * 5. Launch the threads and let the publishers publish the images.
         *
         * Although what this method does is the same for both derived classes of source_base,
         * due to the fact that \ref initialize needs to call \ref getMyNodeHandle and \ref getMyPrivateNodeHandle,
         * which are both pure virtual functions, Source_base cannot call \ref initialize within its own constructor.
         * It has to be called by the derived classes.
         *
         */
        void initialize();
    
    
    private:
        
        /**
         * \brief Dynamical array for depth publishers
         *
         * Will be allocated memory when Source_base::initialize is called. Memory freed in destructor.
         *
         * Raw pointers are prone to memory leakage and illegal access. Shall be safer if std::vector is used.
         */
        ros::Publisher* depth_pub;
        
        
        /**
         * \brief Dynamical array for RGB publishers
         *
         * On how this is used and its caveats, please refer to \ref depth_pub
         */
        ros::Publisher* rgb_pub;
        
        
        /**
         * \brief Message publisher
         *
         * Not used at this moment. In the future might be used as a side-channel for internodal communications.
         */
        ros::Publisher T_pub;
        
        
        /**
         * \brief Configurable parameter
         *
         * If turned on (ture), depth frames will be aligned to the viewpoint of the RGB frames before being transmitted.
         * Turning on alignment causes a non-negligible overhead on CPU time
         * but liberates the user from manually cropping the depth images.
         *
         * Default value: false.
         */
        bool enable_align = false;
        
        
        /**
        * \brief Configurable parameter
        *
        * Default number of channel diversity (in other words, # of threads). It is equal to the size of the dynamically-allocated array Source_base::depth_pub and Source_base::rgb_pub, and the size of vector Source_base::thread_list.
        *
        * Default value: 1.
        */
        int diversity = 1;
        
        
        /**
        * \brief Configurable parameter
        *
        * FPS value for the camera. It can only be 15, 30, 60 or 90. All other values default to the value of camera::Default_FPS.
        * When set to 90, RGB channel will run at 60 Hz which is its maximum performance.
        *
        * Default value: camera::Default_FPS.
        *
        * \see The datasheet for the device:
        * https://dev.intelrealsense.com/docs/intel-realsense-d400-series-product-family-datasheet
        *
        * [May 21st, 2020] It is said that the DS 435 device can be configured to capture as high as 300 frames per second
        * with a resolution of 100*840. Please checkout this website:
        * https://dev.intelrealsense.com/docs/high-speed-capture-mode-of-intel-realsense-depth-camera-d435
        */
        int FPS = camera::Default_FPS;
        
        
        /**
         * \brief Post-processing blocks to align the depth frame to the RGB frame. Each thread owns one.
         *
         * rs2::align is a post-processing block that can align one channel to the view point of another channel.
         *
         * \see How post-processing blocks work on depth frames:
         * https://dev.intelrealsense.com/docs/depth-post-processing
         */
        std::vector<rs2::align> align_to_color;
        
        
        /**
         * \brief Thread container for all depth and RGB publishers
         *
         * The size of Source_base::thread_list depends on the value of Source_base::diversity.
         *
         * For each pair of depth publisher and RGB publisher, they share a thread and publish their messages together
         */
        std::vector<boost::thread> thread_list;
        
        
        /**
         * \brief Serial number counter for frames that it obtained from the camera
         *
         * This is the counter that counts how many frames it has obtained from the camera (and equally, how many frames it published).
         * It does not care about the timestamp of each frame, therefore it cannot be used to identify frame loss.
         *
         * Each image message being transmitted in the ROS infrastructure is labelled with this serial number;
         * the value is stored at sensor_msgs::Image::header::frame_id.
         *
         * \see ROS documentation for sensor_msgs::Image:
         * http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Image.html
         */
        uint32_t seq = 0;
        
        
        /**
         * \brief Mutex to protect Source_base::seq against multi-thread access
         *
         * Each thread has to lock Source_base::seq_mutex before manipulating the serial number counter Source_base::seq
         */
        boost::mutex seq_mutex;
        
        
        rs2::pipeline p;///<Realsense pipeline
        
        rs2::config c;///<Realsense device configurations
        
        
        /**
         * \brief Define as many RGB and depth publishers as \ref Source_base::diversity
         * @return Reference to self, enabling method chaining
         */
        Source_base& definePublishers();
        
        
        /**
         * \brief Turn on the camera
         *
         * Fetches parameters align and FPS from ROS parameter server and starts the camera.
         * @return Reference to self, enabling method chaining
         */
        Source_base& initCamera();
    
        
        /**
         * \brief Publish the moment this method is called as a parameter. Units in microsecond.
         *
         * Get the current Unit-time in microseconds and publish it to ROS parameter server in the name of "rs_start_time". This parameter is currently not used though.
         * @return Reference to self, enabling method chaining
         * \see helper::get_time_stamp_str
         */
        Source_base& setParamTimeOfStart();
        
        
        /**
         * \brief The core routine that the source does - to interface with the camera and transmit images
         *
         * The routine will wait blockingly for the next frame from the camera (and process it) and copy it to an instance of ROS's sensor_msgs::Image. The message is then published.
         *
         * On the selection of which publisher to use, every thread has the choice to use any of rgb_pub and depth_pub. It determines which one to use by calculating the the image's serial number modulo the value of Source_base::diversity : ```index = local_seq  % diversity```. This can cause undesired behavior when two threads "collide" on the same publisher. It might be wiser to strictly assign each thread a specific RGB and depth publisher to use.
         *
         * kernelRoutine is designed to be run by a managed thread in an infinite loop.
         * The first line of kernelRoutine is a thread interruption point. Whenever the thread receives an interruption, it will throw an exception and return from here.
         * Here is the exemplary usage of kernelRoutine:
         * ```
         *  //define the infinite loop and the exit mechanism
         *  auto f=[&](){
         *      try {while(true) kernelRouine();}
         *      catch(boost::thread_interrupted&) {return;}
         *  }
         *
         *  //launch the thread
         *  boost::thread t(f);
         *
         *  //...
         *
         *  //terminate the thread
         *  t.interrupt();
         *  t.join();
         * ```
         * \see Documentation on boost::thread's thread management:
         * https://www.boost.org/doc/libs/1_58_0/doc/html/thread/thread_management.html
         */
        void kernelRoutine();
        
        
        /**
         * \brief Launch as many threads as Source_base::diversity has defined
         *
         * Each thread runs kernelRoutine in an infinite loop. They can be interrupted and then joined by calling boost::thread::interrupt() and boost::thread::join() upon them.
         *
         * \see kernelRoutine for an example on how to use lambdas to generate an infinite loop which executes kernelRoutine
         */
        void launchThreads();
        
    };
    
    
    class Source_nodelet : public Source_base, public nodelet::Nodelet {
    protected:
        virtual ros::NodeHandle& getMyNodeHandle() override;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
    
    public:
        //onInit is an override of nodelet::Nodelet::onInit()
        virtual void onInit() override;
        
        virtual ~Source_nodelet();
    };
    
    class Source_independent : public Source_base {
    public:
        Source_independent() {
            selfInit();
        }
        
        virtual ~Source_independent();
    
    private:
        ros::NodeHandle nh;
        ros::NodeHandle nph;
    
    protected:
        //equivalent of method void nodelet::Nodelet::onInit(), but since Source_independent is not dereived from Nodelet
        //we just have to call selfInit() in constructor
        void selfInit();
        
        virtual ros::NodeHandle& getMyNodeHandle() override;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
        
    };
    
    
    class Drain_base {
    public:
        
        Drain_base() :
                depth_sub(nullptr), rgb_sub(nullptr) {
//            NODELET_INFO("camera drain node constructed\n");
            std::cout << ("camera drain node base constructed\n");
        };
        
        virtual ~Drain_base();
    
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
        
        virtual Drain_base& define_subscribers();
        
        virtual Drain_base& create_directories();
        
        virtual Drain_base& save_image(cv_bridge::CvImageConstPtr, std_msgs::Header, unsigned int);
        
        void initialize();
    };
    
    
    class Drain_nodelet : public Drain_base, public nodelet::Nodelet {
    protected:
        virtual ros::NodeHandle& getMyNodeHandle() override;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
    
    public:
        //Drain_nodelet::onInit is override to nodelet::Nodelet::onInit
        virtual void onInit() override;
        
        virtual ~Drain_nodelet();
        
    };
    
    class Drain_independent : public Drain_base {
    public:
        Drain_independent() {
            selfInit();
        }
        
        virtual ~Drain_independent();
    
    private:
        ros::NodeHandle nh;
        ros::NodeHandle nph; //private handle
        
        //equivalent of method void nodelet::Nodelet::onInit(), but since Source_independent is not dereived from Nodelet
        //we just have to call selfInit() in constructor
        void selfInit();
    
    protected:
        virtual ros::NodeHandle& getMyNodeHandle() override;
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
        
    };
    
    
}


#endif
