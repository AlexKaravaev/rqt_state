#include <ros/ros.h>
#include <ros/network.h>
#include <ros/master.h>
#include <std_msgs/Int64.h>
#include <string>
#include <QDebug>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/rqt_state/qnode.hpp"

namespace rqt_state
{

QNode::QNode(int argc, char **argv) : init_argc(argc),
                                      init_argv(argv)
{
}

QNode::~QNode()
{
    if (ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc, init_argv, "rqt_state");
    if (!ros::master::check())
    {
        return false;
    }
    this->m_nh.reset(new ros::NodeHandle());
    ros::start();

    start();
    return true;
}

void QNode::run()
{
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/** @brief Add topic_widget_box for monitoring
 *  @param QT Object name
 *  @param topic name for subscribing
 *  @param ENUM msg type for callback
 **/
void QNode::addTopicWidget(const QString &object_name, const QString &topic_name, msg_type msg_type)
{
    if (msg_type == INT_64)
    {
        ros::Subscriber sub = m_nh->subscribe<std_msgs::Int64>(topic_name.toStdString(), 10, boost::bind(&QNode::int64TopicCallback, this, _1, object_name));
        m_topic_subs.push_back(sub);
    }

    if (msg_type == STATS_DROPPED_MSGS ||
        msg_type == STATS_HZ ||
        msg_type == STATS_BANDWIDTH)
    {
        ros::Subscriber sub = m_nh->subscribe<rosgraph_msgs::TopicStatistics>(std::string("/statistics"), 10, boost::bind(&QNode::statisticsCallback, this, _1, object_name, topic_name, msg_type));
        m_topic_subs.push_back(sub);
    }
}

/** @brief callback for Int64 message
 **/
void QNode::int64TopicCallback(const std_msgs::Int64::ConstPtr &msg,
                               const QString &object_name)
{
    QPair<QString, QString> topic_msg;
    qDebug() << "Callback: " << msg->data;
    topic_msg.first = object_name;
    topic_msg.second = QString::number(msg->data);
    Q_EMIT loggingUpdated(topic_msg);
}

/** @brief callback for statistics message. Grep topic name, delivred_msgs and dropped_msgs
 **/
void QNode::statisticsCallback(const rosgraph_msgs::TopicStatistics::ConstPtr &msg,
                               const QString &object_name,
                               const QString &topic_name,
                               const msg_type &msg_type)
{
    // Check if the topic mathes the one we want to monitor
    if (msg->topic == topic_name.toStdString())
    {

        QPair<QString, QString> qt_msg;
        qt_msg.first = object_name;
        if (msg_type == STATS_DROPPED_MSGS)
            qt_msg.second = QString::number(msg->dropped_msgs);
        if (msg_type == STATS_HZ)
        {

            ros::Time start = msg->window_start;
            ros::Time stop = msg->window_stop;

            qt_msg.second = QString::number(msg->delivered_msgs / (stop - start).toSec());
            qDebug() << "HZ: " << QString::number(msg->delivered_msgs / (stop - start).toSec());
        }
        if (msg_type == STATS_BANDWIDTH)
        {

            ros::Time start = msg->window_start;
            ros::Time stop = msg->window_stop;

            qt_msg.second = QString::number(msg->traffic / (stop - start).toSec());
            qDebug() << "BD: " << QString::number(msg->traffic / (stop - start).toSec());
        }
        Q_EMIT loggingUpdated(qt_msg);
    }
}

} // namespace rqt_state
