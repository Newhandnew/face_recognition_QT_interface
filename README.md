Face Recognition Interface
=======

### Integrate QT interface
[ROS Face Recognition](https://github.com/Newhandnew/face_recognition) fork from [the ROS wiki](http://wiki.ros.org/face_recognition)

The original server didn't have interface to use, so I made a basic one.

### Usb Cam

To test on my laptop, [ROS USB Cam](https://github.com/Newhandnew/usb_cam) is needed.

Usb-cam-node is forked from [the ROS wiki](http://wiki.ros.org/usb_cam).

Add a new launch file for remape the topic name.

### Usage

1. cd catkin_ws/src  (your catkin workspace).
2. git clone https://github.com/Newhandnew/face_recognition   (face recognition)
3. git clone https://github.com/Newhandnew/face_recognition_QT_interface.git    (face recognition interface)
4. git clone https://github.com/Newhandnew/usb_cam
5. cd face_recognition
6. ./face_recognition.sh  (because the original one use actionlib as service node, I write a quick start script)
7. open a new terminal window
8. rosrun face_recognition_interface face_recognition_interface

Now you can use the interface to recognize faces.



