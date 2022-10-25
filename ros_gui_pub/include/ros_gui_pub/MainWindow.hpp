/**
 * @file /include/ros_gui_pub/main_window.hpp
 *
 * @brief Qt based gui for ros_gui_pub.
 *
 * @date November 2010
 **/
#ifndef ros_gui_pub_MAIN_WINDOW_H
#define ros_gui_pub_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_MainWindow.h"
#include "QNode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ros_gui_pub {

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

	void on_button_connect_clicked(bool check );
	
    void updateLoggingView(); // no idea why this can't connect automatically
	
	void updateLoggingView_sub();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace ros_gui_pub

#endif // ros_gui_pub_MAIN_WINDOW_H
