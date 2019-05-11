# cam(era) (m)ultiplex
This project is developed in catkin workspace, aimed at utilizing **multi-thread processing** while taking advantage from the `nodelet`. Data are send via **channel diversity** on the granularity of messages. 

***

Nodelets under a same nodelet managers share the common memory space, which eliminates the need to copy message data from one node to the other. This is especially useful when the computation is heavy. This sharing of memory is enabled by using `SharePtr`. However, due to this limitation it becomes impossible to run concurrently multiple instances of the callback function on a topic: there is only one "slot" for the message to be processed, so the messages have to be processed in serial.

To solve this problem, I create multiple topics, and rotationally send messages different channels. This is similar to how antenna diversity works. As long as the subscriber has more open channels than the publisher, no messages will be missed due to diversity.

Based on observation, two channels shall be enough for 60 FPS 640*480 pixel resolution.  

