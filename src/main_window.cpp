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
#include "../include/rqt_state/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rqt_state {

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


    // For closing
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    qRegisterMetaType<QPair<QString,QString>>("QPair<QString,QString>");
    // Logging
    topic_name_widget["topic_widget"] = ui.topic_viewer;
    qnode.add_topic_widget(ui.topic_viewer->objectName());
    QObject::connect(&qnode, SIGNAL(loggingUpdated(QPair<QString,QString>)), this, SLOT(updateTopicBox(QPair<QString, QString>)));
    qnode.init();

}

void MainWindow::updateTopicBox(QPair<QString, QString> topic_msg)
{
    topic_name_widget[topic_msg.first]->setText(topic_msg.second);
}
MainWindow::~MainWindow() {}



void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace rqt_state

