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
#include "../include/ros_gui_pub/MainWindow.hpp"
// #include "QNode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_gui_pub {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this);
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); 
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	
	ui.view_logging->setModel(qnode.loggingModel());
	ui.view_logging_sub->setModel(qnode.loggingModel_sub());

    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
	QObject::connect(&qnode, SIGNAL(loggingUpdated_sub()), this, SLOT(updateLoggingView_sub()));
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


void MainWindow::on_button_connect_clicked(bool check ) {
	if ( !qnode.init() ) {
		showNoMasterMessage();
	} else {
		ui.button_connect->setEnabled(false);
	}
}


void MainWindow::updateLoggingView() {
    ui.view_logging->scrollToBottom();
}

void MainWindow::updateLoggingView_sub() {
	ui.view_logging_sub->scrollToBottom();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace ros_gui_pub

