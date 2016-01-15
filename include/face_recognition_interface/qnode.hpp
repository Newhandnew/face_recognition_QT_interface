/**
 * @file /include/face_recognition_interface/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef face_recognition_interface_QNODE_HPP_
#define face_recognition_interface_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <face_recognition/FaceRecognitionAction.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <QtGui>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace face_recognition_interface {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
	void sendCommand(int orderID, const char *argument);
	void setThreshold(int setting);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void imageUpdated(QImage currentImage);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	ros::Publisher face_recognition_command;
	ros::Subscriber face_recognition_feedback;
	ros::Subscriber image_receiver;
    QStringListModel logging_model;
    void feedbackCB(face_recognition::FaceRecognitionFeedback feedback);
    void imageCB(const sensor_msgs::ImageConstPtr& msg);
};

}  // namespace face_recognition_interface

#endif /* face_recognition_interface_QNODE_HPP_ */
