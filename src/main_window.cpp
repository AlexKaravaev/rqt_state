#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rqt_state/main_window.hpp"

namespace rqt_state
{

using namespace Qt;

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), m_qnode(argc, argv)
{
    ui.setupUi(this);                                                                    // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    // For closing
    QObject::connect(&m_qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    qRegisterMetaType<QPair<QString, QString>>("QPair<QString,QString>");

    // Init ROS node
    m_qnode.init();

    // Add QtTextBox object to map with key==object text name
    m_qt_name_to_topic_name["topic_viewer"] = ui.topic_viewer;
    m_qt_name_to_topic_name["topic_viewer_2"] = ui.topic_viewer_2;
    m_qt_name_to_topic_name["topic_viewer_3"] = ui.topic_viewer_3;

    m_qnode.addTopicWidget(ui.topic_viewer->objectName(), ui.topic_viewer->property("topic_name").toString(), STATS_DROPPED_MSGS);

    m_qnode.addTopicWidget(ui.topic_viewer_2->objectName(), ui.topic_viewer->property("topic_name").toString(), STATS_HZ);

    m_qnode.addTopicWidget(ui.topic_viewer_3->objectName(), ui.topic_viewer->property("topic_name").toString(), STATS_BANDWIDTH);

    // Connect to ROS signal
    QObject::connect(&m_qnode, SIGNAL(loggingUpdated(QPair<QString, QString>)), this, SLOT(updateTopicBox(QPair<QString, QString>)));
}

void MainWindow::updateTopicBox(QPair<QString, QString> topic_msg)
{
    qDebug() << "Callback: " << topic_msg.first;
    if (m_qt_name_to_topic_name.contains(topic_msg.first))
        m_qt_name_to_topic_name[topic_msg.first]->setText(topic_msg.second);
    else
        ROS_ERROR("Dictionary does not contain object with given name: %s", topic_msg.first.toStdString().c_str());
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

} // namespace rqt_state
