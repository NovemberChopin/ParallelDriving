
#include <QtGui>
#include <QApplication>
#include <QMetaType>
#include "../include/parallel_driving/main_window.hpp"

int main(int argc, char **argv) {

    QApplication app(argc, argv);
    parallel_driving::MainWindow w(argc,argv);
    // w.show();

    qRegisterMetaType<cv::Mat>("cv::Mat");
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

	return app.exec();
}
