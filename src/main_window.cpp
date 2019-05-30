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

    // Set timer fallback
    tmr = new QTimer();
    tmr->setInterval(2000);
    tmr->start();

    // Init ROS node
    m_qnode.init();

    // Add /udp/topic_out boxes to ros
    // Map object to it's name
    m_qt_name_to_object[ui.topic_viewer->objectName()] = ui.topic_viewer;
    m_qt_name_to_object[ui.topic_viewer_2->objectName()] = ui.topic_viewer_2;
    m_qt_name_to_object[ui.topic_viewer_3->objectName()] = ui.topic_viewer_3;

    m_qnode.addTopicWidget(ui.topic_viewer->objectName(), ui.topic_viewer->property("topic_name").toString(), STATS_DROPPED_MSGS);
    m_qnode.addTopicWidget(ui.topic_viewer_2->objectName(), ui.topic_viewer_2->property("topic_name").toString(), STATS_HZ);
    m_qnode.addTopicWidget(ui.topic_viewer_3->objectName(), ui.topic_viewer_3->property("topic_name").toString(), STATS_BANDWIDTH);

    // Add /int boxes to ros
    // Map object to it's name
    m_qt_name_to_object[ui.topic_viewer_4->objectName()] = ui.topic_viewer_4;
    m_qt_name_to_object[ui.topic_viewer_5->objectName()] = ui.topic_viewer_5;
    m_qt_name_to_object[ui.topic_viewer_6->objectName()] = ui.topic_viewer_6;

    m_qnode.addTopicWidget(ui.topic_viewer_4->objectName(), ui.topic_viewer_4->property("topic_name").toString(), STATS_DROPPED_MSGS);
    m_qnode.addTopicWidget(ui.topic_viewer_5->objectName(), ui.topic_viewer_5->property("topic_name").toString(), STATS_HZ);
    m_qnode.addTopicWidget(ui.topic_viewer_6->objectName(), ui.topic_viewer_6->property("topic_name").toString(), STATS_BANDWIDTH);

    // Connect to ROS signal
    QObject::connect(&m_qnode, SIGNAL(loggingUpdated(QPair<QString, QString>)), this, SLOT(updateTopicBox(QPair<QString, QString>)));

    //Connect to timer
    QObject::connect(tmr, SIGNAL(timeout()), this, SLOT(resetTopicBoxes()));
}

/** @brief reset all topic boxes once in 2 seconds,in order to see, when no msg received on subscriber
 **/
void MainWindow::resetTopicBoxes()
{
    for (auto box : m_qt_name_to_object.values())
    {
        box->setText(QString("-"));
    }
}

void MainWindow::updateTopicBox(QPair<QString, QString> topic_msg)
{
    // Check if qt_object with given name is present
    if (m_qt_name_to_object.contains(topic_msg.first))
        m_qt_name_to_object[topic_msg.first]->setText(topic_msg.second);
    else
        ROS_ERROR("Dictionary does not contain object with given name: %s", topic_msg.first.toStdString().c_str());
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

} // namespace rqt_state
