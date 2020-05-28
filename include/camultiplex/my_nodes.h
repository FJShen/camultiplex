#ifndef CAMULTIPLEX__MY_NODES
#define CAMULTIPLEX__MY_NODES

#include "ros/ros.h"
#include <librealsense2/rs.hpp>
#include "camultiplex/helper.h"
#include <nodelet/nodelet.h>
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <boost/thread.hpp>
#include <vector>

#define RGB 0x0000
#define DEPTH 0x0001

//todo: re-evaluate access specifiers for each method and member variable

/**
 * \namespace camera
 * \brief All ROS nodes for the package "camultiplex" are defined in this namespace
 *
 * ## Purpose of design
 *
 * When using the package "camultiplex" in practice to collect images from an Intel RS435 camera and store them in any filesystem,
 * usually two nodes are involved: 1) a "source" that interfaces with the camera device, 2) a "drain" that interfaces with the file system.
 * The "source" transforms the image frames into ROS-compatible format, transmits the packets across the ROS middleware infrastructure;
 * the "drain" receives and restores the packets, then store them to the filesystem with .png or .jpg format.
 *
 * Furthermore, depending on physical setup one may wish to run both nodes 1) on a single host or 2) on different hosts within a network.
 *
 * In the case two nodes are run on the same host, it is more efficient to run them as <em>nodelets</em> which share the same memory space.
 * This eliminates the overhead cause by TCP communication that is ubiquitous among any two "non-nodelet" nodes.
 *
 * When two nodes are not run on the same host, however, TCP (or UDP) connection is the only solution. Nodelets are not possible.
 *
 * Based on the reasoning described above, four different classes can be defined:
 *
 * 1. Source_nodelet
 * 2. Drain_nodelet
 * 3. Source_independent (non-nodelet)
 * 4. Drain_independent (non-nodelet)
 *
 *   It would turn out that for the drain- and the source-side, respectively, the difference between codes for the nodelet version
 *   and the independent version is very small but subtle. In order to promote code re-usability,
 *   two base classes that serve as common "anscestors" should be added:
 *
 * 5. Source_base
 * 6. Drain_base
 *
 * ## Methodology of design
 *
 * The subtle difference between a nodelet and a non-nodelet is merely how they obtain their node handles.
 * The base classes will implement 99% essential functionality but leave the methods that returns
 * their (public) node handle and private node handle as pure virtual functions to be defined. Source_base and Drain_base, therefore,
 * are *abstract* classes that cannot be instantiated.
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
    const std::string Default_path = "/media/nvidia/ExtremeSSD";
    const int Default_drain_diversity = 3;
    const int Default_source_diversity = 3;
    
    
    /**
     * \class Source_base
     * \brief Interfaces with the camera and publishes the image frames in the form of sensor_msgs::Image
     *
     * ## Configurable parameters
     * 1. **diversity**: number of topics that source publishes to
     * 2. **FPS**: FPS of camera
     * 3. **align**: to align depth frames to the viewpoint of RGB frames
     *
     * ## Usage
     * Source_base is an abstract class that provides nearly all functionality needed by a source node which interfaces with the camera.
     * Its two pure virtual functions, \ref getMyNodeHandle and \ref getMyPrivateNodeHandle need to be implemented by derived class depending on whether they are nodes or nodelets.
     */
    class Source_base {
    
    public:
        /**
         * \brief Constructor
         *
         * depth_pub and rgb_pub are initialized as nullptr
         */
        Source_base();
        
        
        /**
         * \brief Destructor
         *
         * All threads from \ref thread_list are joined, the camera is stopped,
         * the depth_pub and rgb_pub arrays are deleted.
         */
        virtual ~Source_base();
    
    protected:
        
        virtual ros::NodeHandle& getMyNodeHandle() = 0; ///<virtual function to fetch node handle
        
        virtual ros::NodeHandle& getMyPrivateNodeHandle() = 0; ///<virtual function to fetch private node handle
        
        
        /**
         * \brief Initialize and start the source node.
         *
         * This method obtains the parameters (FPS, align, diversity) from ROS parameter server and does the following in order:
         * 1. Start the camera with the required FPS and alignment setting
         * 2. Populate rgb_pub and depth_pub with the required amount of publishers
         * 3. Set up a new parameter called "rs_start_time". The drain node will have access to this parameter (though it is currently not utilized).
         * 4. Instantiate matching number of alignment processing blocks and put them into the vector \ref align_to_color
         * 5. Launch the threads and let the publishers publish the images.
         *
         * Although what this method does is the same for both derived classes of source_base,
         * due to the fact that \ref initialize needs to call \ref getMyNodeHandle and \ref getMyPrivateNodeHandle,
         * which are not implemented by the base class, Source_base cannot call \ref initialize within its own constructor.
         * This method has to be called by the derived classes.
         */
        void initialize();
    
    
    private:
        
        /**
         * \brief Dynamical array for depth publishers
         *
         * Depth images are in MONO16 format.
         *
         * Will be allocated memory when Source_base::initialize is called. Memory freed in destructor.
         *
         * Raw pointers are prone to memory leakage and illegal access. Shall be safer if std::vector is used.
         */
        ros::Publisher* depth_pub;
        
        
        /**
         * \brief Dynamical array for RGB publishers
         *
         * RGB images are in RGB8 format. The receiver will need to convert this format into BGR8 before calling cv::imwrite. See \ref Drain_base::drain_rgb_callback.
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
        * Default number of channel diversity (in other words, # of threads). It is equal to the size of the dynamically-allocated arrays Source_base::depth_pub and Source_base::rgb_pub, and the size of vector Source_base::thread_list.
        *
        * Default value: camera::Default_source_diversity.
        */
        int diversity = camera::Default_source_diversity;
        
        
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
         * Since this variable will be modified by many threads, a mutex is needed.
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
         * \brief Publish as a parameter the time that this method is called. Unit in microseconds.
         *
         * Get the current Unix-time (since 01/01/1970) in microseconds and publish it to ROS parameter server in the name of "rs_start_time". This parameter is currently not used anywhere.
         * @return Reference to self, enabling method chaining
         * \see helper::get_time_stamp_str
         */
        Source_base& setParamTimeOfStart();
        
        
        /**
         * \brief The core routine that the source does - to interface with the camera and transmit images
         *
         * The routine will wait blockingly for the next frame from the camera (and process it) and copy it to an instance of ROS's sensor_msgs::Image. The message is then published. Depth images are encoded in MONO16; RGB images are encoded in RGB8.
         *
         * On the selection of which publisher to use, every thread has the choice to use any of rgb_pub and depth_pub. It determines which one to use by calculating the the image's serial number modulo the value of Source_base::diversity : ```index = local_seq  % diversity```. This can cause undesired behavior when two threads "collide" on the same publisher. It might be wiser to strictly assign each thread a specific RGB and depth publisher to use.
         *
         * kernelRoutine is designed to be run by a managed thread in an infinite loop.
         * The first line of kernelRoutine is a thread interruption point, therefore whenever the thread receives an interruption, it will throw an exception and return from here.
         * Here is the exemplary usage of kernelRoutine:
         * ```
         * #include "camultiplex/my_nodes.h"
         * #include <boost/thread.hpp>
         *
         *  //define the infinite loop and the exit mechanism; this is the lambda expression
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
         * \see kernelRoutine for an example on how to generate a lambda expression to execute kernelRoutine in an infinite loop
         */
        void launchThreads();
        
    };
    
    /**
     * \class Source_nodelet
     * \brief The nodelet version of source
     */
    class Source_nodelet : public Source_base, public nodelet::Nodelet {
    protected:
        /**
         * @return The (public) multi-thread handle for this nodelet.
         * \see The thread model for ROS nodelets:
         * http://wiki.ros.org/nodelet#Threading_Model
         */
        virtual ros::NodeHandle& getMyNodeHandle() override;
        
        
        /**
         * @return The private multi-thread handle for this nodelet.
         * \see The thread model for ROS nodelets:
         * http://wiki.ros.org/nodelet#Threading_Model
         *
         * \see ROS naming conventions: http://wiki.ros.org/Names
         */
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
    
    public:
        /**
         * \brief onInit is an override of nodelet::Nodelet::onInit()
         *
         * Calls Source_base::initialize
         */
        virtual void onInit() override;
        
        
        /**
         * \brief Deletes all parameters that the source node had added to ROS parameter server in Source_base::initialize
         */
        virtual ~Source_nodelet();
    };
    
    
    /**
    * \class Source_independent
    * \brief The non-nodelet version of source
    */
    class Source_independent : public Source_base {

    public:
        ///calls selfInit()
        Source_independent() {
            selfInit();
        }
    
    
        ///Deletes all parameters that the source node had added to ROS parameter server in Source_base::initialize
        virtual ~Source_independent();
        
    protected:
        ///Returns \ref nh
        virtual ros::NodeHandle& getMyNodeHandle() override;
    
        
        ///Returns \ref nph
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
        
    private:
        /**
         * \brief The node's handle
         *
         * For the (public) node handle of an independent node, it does not need to be explicitly defined. It gets initialized (by calling ros::NodeHandle()) automatically during construction.
         */
        ros::NodeHandle nh;
    
    
        /**
         * \brief The node's private handle
         *
         * For a private node handle, any method called upon it resolves to its own "name scope". We want the "name scope" of this private handle to be the node's own name, therefore we have to explicitly call ros::NodeHandle(const std::string& namespace) and give it a name:
         * ```
         * npg = ros::NodeHandle(ros::this_node::getName());
         * ```
         * before calling Source_base::initialize().
         */
        ros::NodeHandle nph;
    
        //equivalent of method void nodelet::Nodelet::onInit(), but since Source_independent is not dereived from Nodelet
        //we just have to call selfInit() in constructor
        /**
         * \brief Counterpart of Source_nodelet::onInit() for the non-nodelet
         *
         * This method sets up the private node handle and then calls Source_base::initialize
         *
         * \see nph
         */
        void selfInit();

    };
    
    /**
     * \class Drain_base
     * \brief Interfaces with the file system. Converts sensor_msgs::Image into .jpg and .png format and stores them as images.
     *
     * ## Configurable parameters
     * 1. **diversity**: number of topics that drain subscribes to
     * 2. **base_path**: path to create folder
     *
     * ## Usage
     * Source_drain is an abstract class that provides nearly all functionality needed by a source node which interfaces with the camera.
     * Its two pure virtual functions, getMyNodeHandle and getMyPrivateNodeHandle need to be implemented by
     * derived classes.
     */
    class Drain_base {
    public:
        /**
         * \brief Initializes depth_sub and rgb_sub as nullptr
         */
        Drain_base() :
                depth_sub(nullptr), rgb_sub(nullptr) {
            std::cout << ("camera drain node base constructed\n");
        };
        
        
        /**
         * \brief Deletes depth_sub and rgb_sub
         */
        virtual ~Drain_base();
    
    protected:
        /**
         * \brief Initialize and start the drain node
         *
         * The method obtains parameters (diversity, base_path) from ROS parameter server and does the following in order:
         * 1. Create a new directory under the path specified by base_path. The directory is named in the format of [yyyy_m_d_h_m]
         * 2. Define as many subscribers as required by diversity. Unless you want to make the drain miss any frame transmitted by the source,
         * drain diversity should be no less than source diversity.
         */
        void initialize();
        
        
        /**
         * \brief Pure virtual function to fetch node handle
         * @return (Public) node handle
         */
        virtual ros::NodeHandle& getMyNodeHandle() = 0;
    
        
        /**
         * \brief Pure virtual function to fetch private node handle
         * @return Private node handle
         */
        virtual ros::NodeHandle& getMyPrivateNodeHandle() = 0;
        
    private:
        /**
         * \brief Dynamic array of depth channel subscribers
         */
        ros::Subscriber* depth_sub;
        
        
        /**
         * \brief Dynamic array of RGB channel subscribers
         */
        ros::Subscriber* rgb_sub;
        
        
        /**
         * \brief Manages the periodic display of receiver information
         *
         * timerCallback is set to be called every 5 seconds to display information about the number of received frames.
         *
         * \see timerCallback
         */
        ros::Timer timer;
        
        
        /**
         * \brief Count the number of received depth frames and estimate lost of frames during transmission
         *
         * Call helper::Counter::updateSeq upon the serial number of every received frame
         * in drain_rgb_callback or drain_depth_callback.
         *
         * Example:
         * ```
         * void callback(const sensor_msgs::Image::ConstPtr& msg){
         *      //process images
         *      //...
         *
         *      //update counter with serial number
         *      depth_counter.updateSeq(std::stoul(msg->header.frame_id));
         * }
         * ```
         *
         * helper::Counter has an in-built mutex to ensure thread safety.
         * No need for the caller to do additional protection.
         * \see helper::Counter
         */
        helper::Counter depth_counter;
    
        
        /**
         * \brief Count the number of received RGB frames and estimate lost of frames during transmission
         *
         * \see depth_counter
         */
        helper::Counter rgb_counter;
        
        
        /**
         * \brief The path where the folder will be created
         * \see create_directories
         */
        std::string base_path = camera::Default_path;
        
        
        /**
         * \brief Path of the created directory (the directory that contains the sub-directories rgb_images and depth_images)
         * \see create_directories
         */
        std::string folder_path;
        
        
        /**
         * \brief Drain diversity
         *
         * Default value: camera::Default_drain_diversity
         */
        int diversity = camera::Default_drain_diversity;
    
        
        /**
         * \brief Create a new directory under base path
         *
         * The new directory is named with the format "year_month_date_hour_minute" based on the host's local time setting. E.g. "2020_5_22_15_4" means the directory was created at 15:04, May 22nd, 2020
         *
         * Under this newly created director, two sub-directories - "rgb_images" and "depth_images" - are also created.
         *
         * After creating directories, the current path is set to be \ref folder_path.
         * @return Reference to self, allowing method chaining.
         */
        Drain_base& create_directories();
        
        
        /**
         * \brief Define as many subscribers as required by diversity
         *
         * The following topics will be subscribed to by the drain:
         *
         * camultiplex/depthx, where x=0, 1, ..., diversity-1
         *
         * camultiplex/RGBx, where x=0, 1, ..., diversity-1
         *
         * @return Reference to self, allowing method chaining.
         */
        Drain_base& define_subscribers();
    
        
        /**
         * \brief Callback function to process depth channel images
     
         * @param msg A shared_ptr to the received message.
         * @param channel_num Index of the topic being subscribed to. Currently unused.
         * \see drain_rgb_callback
         */
        void drain_depth_callback(const sensor_msgs::Image::ConstPtr& msg, int channel_num);
    
        
        /**
         * \brief Callback function to process RGB channel images
         *
         * Callback function uses cv_bridge::toCvShare to convert the received sensor_msgs::Image into a read-only reference to an instance of OpenCV Mat
         *
         * Another choice is to call cv_bridge::toCvCopy instead of toCvShare; this allows the callback function to do mutations on the CV image.
         *
         * The second parameter of cv_bridge::toCvShare - encoding - infers the desired encoding of the image data. If left empty, the returned CvImage has the same encoding as the input image.
         *
         * In the case of our RGB channel image, it arrives from the source encoded in RGB8 format (24 bits); however when storing the image with cv::imwrite, only BGR8 format is allowed. Therefore "bgr8" is passed in.
         *
         * @param msg A shared_ptr to the received message.
         * @param channel_num Index of the topic being subscribed to. Currently unused.
         * \see Turotial for ROS package cv_bridge on how to convert between ROS images and CV images:
         * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
         * \see ROS publisher and subscribers
         * http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
         */
        void drain_rgb_callback(const sensor_msgs::Image::ConstPtr& msg, int channel_num);
    
        
        /**
         * \brief Save image to directory.
         * Depth images (encoded in 16-bit unsigned) are saved in PNG format.
         * RGB images (encoded in BGR8 order) are saved in JPG format.
         * @param cv_ptr CVImageConstPtr as generated in rgb_ or depth_callback.
         * @param header Header of the image message.
         * @param channel Specifies whether the image is depth or RGB. Valid values are RGB or DEPTH.
         * @return  Reference to self, allowing method chaining.
         * \see OpenCV cv::imwrite:
         * https://docs.opencv.org/3.4/d4/da8/group__imgcodecs.html
         */
        Drain_base& save_image(cv_bridge::CvImageConstPtr cv_ptr, std_msgs::Header header, unsigned int channel);
    
        
        
        /**
         * \brief Prints the transmission loss estimate.
         * Being called by the timer everytime the timer has expired.
         *
         * Displays the following information when called:
         * ```
         * Drain: Depth received xxx frames, total transmission loss estimate is xxx frames
         * Drain: RGB received xxx frames, total transmission loss estimate is xxx frames
         * ```
         * \see \ref timer
         */
        void timerCallback(const ros::TimerEvent& event);
        
    };
    
    
    /**
     * \class Drain_nodelet
     * \brief The nodelet version of of Drain
     */
    class Drain_nodelet : public Drain_base, public nodelet::Nodelet {
    protected:
        /**
         * @return The (public) multi-thread handle for this nodelet.
         * \see The thread model for ROS nodelets:
         * http://wiki.ros.org/nodelet#Threading_Model
         */
        virtual ros::NodeHandle& getMyNodeHandle() override;
    
    
        /**
         * @return The private multi-thread handle for this nodelet.
         * \see The thread model for ROS nodelets:
         * http://wiki.ros.org/nodelet#Threading_Model
         * \see ROS naming conventions: http://wiki.ros.org/Names
         */
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
    
    public:
        /**
         * \brief Override to nodelet::Nodelet::onInit
         * Calls Drain_base::initialize.
         */
        virtual void onInit() override;
        
        
        ///Deletes all parameters (diversity, base_path) related to the drain.
        virtual ~Drain_nodelet();
        
    };
    
    
    
    /**
     * \class Drain_independent
     * \brief The non-nodelet version of Drain
     */
    class Drain_independent : public Drain_base {
    public:
        /**
         * \brief Constructor
         * Calls selfInit.
         */
        Drain_independent() {
            selfInit();
        }
    
    
        ///Deletes all parameters (diversity, base_path) related to the drain.
        virtual ~Drain_independent();

    protected:
        /// Returns \ref nh.
        virtual ros::NodeHandle& getMyNodeHandle() override;
    
    
        ///Returns \ref nph.
        virtual ros::NodeHandle& getMyPrivateNodeHandle() override;
        
    private:
        ros::NodeHandle nh;  ///Public node handle.
        ros::NodeHandle nph; ///Private node handle
        
        
        /**
         * \brief Counterpart of Drain_nodelet::onInit
         * Sets up private node handle then calls Drain_independent::initialize.
         */
        void selfInit();

    };
    
}


#endif
