/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <face_recognition/FRClientGoal.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include "../include/face_recognition_interface/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace face_recognition_interface {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"face_recognition_interface");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    face_recognition_command = n.advertise<face_recognition::FRClientGoal>("/fr_order", 1000);
    // face_recognition_feedback = n.subscribe("/face_recognition/feedback", 10, &QNode::feedbackCB, this);
    image_receiver = n.subscribe("/face_recognition/image", 1, &QNode::imageCB, this);
	start();
	return true;
}

void QNode::sendCommand(int orderID, const char *argument) {
    face_recognition::FaceRecognitionGoal goal;
    goal.order_id = orderID;
    goal.order_argument =  argument;
    face_recognition_command.publish(goal);
}

void QNode::setThreshold(int setting) {
	double threshold = (double)setting / 100;
	ros::param::set("/face_recognition/confidence_value", threshold);
}

void QNode::run() {
	ros::Rate loop_rate(10);
	int count = 0;
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::feedbackCB(face_recognition::FaceRecognitionFeedback feedback) {
	log(Info, std::string("feedback: ")+ feedback.names[0]);
}

void QNode::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  //to synchronize with executeCB function.
  //as far as the goal id is 0, 1 or 2, it's active and there is no preempting request, imageCB function is proceed.
  // cv_bridge::CvImagePtr cv_ptr;
  //convert from ros image format to opencv image format
  // try
  // {
  //   cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  // }
  // catch (cv_bridge::Exception& e)
  // {
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }
  // QImage currentImage(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
  //static QLabel *imageLabel;
  //imageLabel->setPixmap(QPixmap::fromImage(currentImage));
	image = *msg;
  Q_EMIT imageUpdated();
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace face_recognition_interface
