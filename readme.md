The format of this markdown file is best manifested when the file is rendered in HTML format.

# camultiplex
This project is developed in catkin workspace, aimed at utilizing **multi-thread processing** and taking advantage from the **nodelet**. Data are sent via **multiple channels** in the form of ROS messages.

This package can either be configured to run as **nodelets** on a single computer or as independent nodes across a computer network. 

***

Nodelets under a same nodelet manager share the common memory space, which eliminates the need to copy data from one node to the other. This is especially useful when the computation is heavy. This sharing of memory is enabled by using `SharedPtr` on the lower level, a feature of modern C++. 

To prevent the throughput from being throttled by the performance of a single CPU core, multiple threads can be utilized which are expected to run on as many cores as the machine possesses. This is made possible by defining more than one subscriber-publisher pairs, each of them communicating on their own channel (aka. ROS topic). When a source node owns multiple publishers, it rotationally lets different publishers to publish the message, which in turn, appears on different channels. In order to ensure that all messages are received by the drain node, it is necessary to let the drain has as many subscribers as the number of publishers. 

Based on observation, two channels (2\*RGB + 2\*Depth) shall be enough for 60 FPS 640*480 pixel resolution.  

***
## Dependencies
ROS and catkin (comes with ROS)

Boost::thread, Boost::filesystem, Boost::chrono

OpenCV

***

## Compilation of code

<pre>
cd <i>your_catkin_workspace_directory</i>/src
git clone http://github.com/FJShen/camultiplex
git checkout master

cd <i>your_catkin_workspace_directory</i>
catkin_make --pkg camultiplex
</pre>

This is what your directory should typically look like (note that before you call catkin_make, 'src' might be the only directory that you see in the workspace):

```
nvidia@virtual-ubuntu:~/catkin_ws$ tree -L 2
.
├── build
│   └── <i>Some files</i>
├── devel
│   └── <i>Some files</i>
├── install
│   └── <i>Some files</i>
└── src
    ├──<b> camultiplex </b>
    └──<i> other packets </i>
```

***

## Execution of nodes

### I.  Nodelets mode
#### Launchfile method (roslaunch)
**1. bring up everything at once**
```
roslaunch camultiplex launch_everything.launch
```
You need to configure the parameters in the files ```./launch/source.launch``` and ```./launch/drain.launch``` respectively.

By your calling ```roslaunch```, ```roscore``` is automatically called.

---


#### Command Line method (rosrun)
**1. summon the master**
```
roscore
```
**2. bring up nodelet manager**
```
rosrun nodelet nodelet manager __name:=nodelet_manager
```
**3. load camera drain as a nodelet to be managed by nodelet_manager**

```
rosrun nodelet nodelet load camultiplex/drain nodelet_manager _diversity:=2 _base_path:="/media/nvidia/ExtremeSSD"
```

**4. load camera source as a nodelet to be managed by nodelet_manager**
```
rosrun nodelet nodelet load camultiplex/source nodelet_manager _diversity:=2 _FPS:=60 _align:=true
```
---
### II. Independent nodes mode
#### Launchfile method (roslaunch)
**1. bring up camera drain**

```
roslaunch camultiplex independent_drain.launch
```

**2. bring up camera source**

```
roslaunch camultiplex independent_source.launch
```

You need to configure the parameters in the files ```./launch/independent_source.launch``` and ```./launch/independent_drain.launch``` respectively.

By your calling ```roslaunch```, ```roscore``` is automatically called.

---
#### Command Line method (rosrun)
**1. summon the master**
```
roscore
```

**2. bring up camera drain**

```
rosrun camultiplex independent_drain _diversity:=2 _base_path:="/media/nvidia/ExtremeSSD"
```

**3. bring up camera source**

```
rosrun camultiplex independent_source _diversity:=2 _FPS:=60 _align:=true
```

---

#### *Running independent nodes on different hosts in LAN
When running a set of nodes that span across a network (e.g. LAN), ```roscore``` only need to be called on one host (i.e. there is only one master in the network). 

Suppose we have two hosts (computers): H1 (192.168.0.11) and H2 (192.168.0.22); we want the master to be run on H1:

1. On H1 and H2 separately ```export ROS_IP=[own_ip_address]``` (this step might not be needed)

2. On H2 ```export ROS_MASTER_URI=http://[h1_ip_addr]:11311``` (i.e. export ROS_MASTER_URI=http://192.168.0.11:11311). 

3. Call ```roscore``` only on H1.

4. On H1 and H2 separately bring up ROS nodes. Message will be transmitted in the form of TCP (or UDP) packets. 

Note: When running a network of ROS nodes on different hosts, nodelets will not be a choice - it is the shared memory mechanism that makes nodelets work.  

---

## Table of Parameters
| Parameter        | Type   | User configurable? | Where to configure                              | Usage                                    |
|------------------|--------|--------------------|-------------------------------------------------|------------------------------------------|
| *drain*/diversity  | int    | Yes                | Drain side                                    | Number of subscribers that a drain uses (1),(2)   |
| *drain*/base_path  | string | Yes                | Drain side                                      | Path for the drain to store images       |
| *source*/diversity | int    | Yes                | Source side                                     | Number of publishers that a source uses (1),(3)     |
| *source*/FPS       | int    | Yes                | Source side                                     | Frames-per-second of the camera          |
| *source*/align     | bool   | Yes                | Source side                                     | Align depth image to RGB image           |
| /rs_start_time   | string | No                 | Global parameter set by  source node at runtime | Unix time of when the camera was started |

Comments:

1) Drain diversity should be no less than source diversity lest some active topics not be subscribed by the drain

2) The number of subscribers are counted in pairs of RGB-Depth subscribers - therefore 1 RGB subscriber and 1 Depth subscriber are counted as "one" subscriber in the context of diversity. 

3) Referring to (2), the same logic applies for publishers.

---

## Generation of documentation

In the package's directory (camultiplex/) call: 

```rosdoc_lite .```

Doxygen will be called and the documentation mainpage will be located at `camultiplex/doc/html/index.html`


