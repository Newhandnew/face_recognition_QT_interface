/**
 * @file /include/face_recognition_interface/main_window.hpp
 *
 * @brief Qt based gui for face_recognition_interface.
 *
 * @date November 2010
 **/
#ifndef face_recognition_interface_MAIN_WINDOW_H
#define face_recognition_interface_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace face_recognition_interface {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();


	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_btn_start_clicked(bool check);
	void on_btn_new_clicked(bool check);
	void on_scrollBar_threshold_valueChanged(int setting);
	void updateImage();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace face_recognition_interface

#endif // face_recognition_interface_MAIN_WINDOW_H
