/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/face_recognition_interface/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace face_recognition_interface {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

//    QObject::connect(ui.scrollBar_threshold, SIGNAL(valueChanged(int)), ui.display_threshold, SLOT(display(int)));

    // connect to master
    if ( !qnode.init() ) {
        showNoMasterMessage();
    } else {
        close();
    }

    // update image
    QObject::connect(&qnode, SIGNAL(imageUpdated(QImage)), this, SLOT(updateImage(QImage)));
	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
    ui.view_logging->scrollToBottom();
}

void MainWindow::updateImage(QImage image) {
    ui.image_show->setPixmap(QPixmap::fromImage(image));
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

void MainWindow::on_btn_start_clicked(bool check) {
    qnode.sendCommand(1, "none");
}

void MainWindow::on_btn_new_clicked(bool check) {
    QString name = ui.textin_name->displayText();
    qnode.sendCommand(2, name.toUtf8().constData());
    sleep(7);
    qnode.sendCommand(3, "none");
}

void MainWindow::on_scrollBar_threshold_valueChanged(int setting) {
    ui.display_threshold->display(setting);
    qnode.setThreshold(setting);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace face_recognition_interface

