The format of this markdown file is best manifested when the file is rendered in HTML format.

# camultiplex
This project is developed in catkin workspace, aimed at utilizing **multi-thread processing** while taking advantage from the `nodelet`. Data are sent via **channel diversity** on the granularity of messages. 

***

Nodelets under a same nodelet manager share the common memory space, which eliminates the need to copy message data from one node to the other. This is especially useful when the computation is heavy. This sharing of memory is enabled by using `SharePtr`. However, due to this limitation it becomes impossible to run concurrently multiple instances of the callback function on a topic: there is only one "slot" for the message to be processed, so the messages have to be processed in serial.

To solve this problem, I create multiple topics, and rotationally send messages to different channels. This is similar to how antenna diversity works. As long as the subscriber has more open channels than the publisher, no messages will be missed due to diversity.

Based on observation, two channels shall be enough for 60 FPS 640*480 pixel resolution.  

***
## Dependencies
ROS & catkin

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

<pre>
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
</pre>

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

**4 load camera source as a nodelet to be managed by nodelet_manager**
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

You need to configure the parameters in the files ```./launch/independent_source.launch``` and ```./launch/ independent_drain.launch``` respectively.

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

#### *Running nodes on different hosts in LAN
When running a set of nodes that span across a network (e.g. LAN) who need to share information by publishing/subscribing to topics, ```roscore``` only need to be called on one host (i.e. there is only one master in the network). 

Suppose we have two hosts (computers): H1 and H2; we want the master to be run on H1:

1. On H1 and H2 separately ```export ROS_IP=[own_ip_address]``` (this step might not be needed)

2. On H2 ```export ROS_MASTER_URI=http://[h1_ip_addr]:11311```. 

3. Call ```roscore``` only on H1.

4. On H1 and H2 separately bring up ROS nodes. Message will be transmitted in the form of TCP (or UDP) packets. 

Note: When running a network of ROS nodes on different hosts, nodelets will not be a choice - it is the shared memory mechanism that makes nodelets work.  

***

## Table of Parameters
| Parameter        | Type   | User configurable? | Where to configure                              | Usage                                    |
|------------------|--------|--------------------|-------------------------------------------------|------------------------------------------|
| *drain*/diversity  | int    | Yes                | Drain side                                      | Number of threads that a drain uses      |
| *drain*/base_path  | string | Yes                | Drain side                                      | Path for the drain to store images       |
| *source*/diversity | int    | Yes                | Source side                                     | Number of threads that a source uses     |
| *source*/FPS       | int    | Yes                | Source side                                     | Frames-per-second of the camera          |
| *source*/align     | bool   | Yes                | Source side                                     | Align depth image to RGB image           |
| /rs_start_time   | string | No                 | Global parameter set by  source node at runtime | Unix time of when the camera was started |





