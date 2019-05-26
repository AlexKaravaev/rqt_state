/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <ros/master.h>
#include <string>
#include <QDebug>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/rqt_state/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/


namespace rqt_state {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode()
{
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init()
{
	ros::init(init_argc,init_argv,"rqt_state");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	start();
	return true;
}



void QNode::run()
{
	ros::Rate loop_rate(1);
        while (ros::ok())
        {

		std_msgs::String msg;
                updateTopicTable();
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/** @brief Add topic_widget_box for monitoring
 **/
void QNode::add_topic_widget(const QString &topic_widget)
{
    m_topic_widgets.insert(topic_widget);
    //last = topic_widget;
    //last->setText("lol");

}

/** @brief generic function to record time for last msg received on topic
 **/
void QNode::topicCallback(const topic_tools::ShapeShifter::ConstPtr &msg, uint64_t &msg_time)
{
    ros::Time time = ros::Time::now();
    msg_time = 1000 * static_cast<uint64_t>(time.sec) + static_cast<uint64_t>(time.sec);
}

/** @brief Retrieve all current topics
 *  @returns QSet<QString> names for all topics
*/
QSet<QString> QNode::getTopics()
{
    QSet<QString> topics;

    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    for(auto it = topic_infos.begin(); it != topic_infos.end(); it++)
        topics.insert(it->name.c_str());


    return topics;
}

/** @brief Retrieve all current nodes
 *  @returns QSet<QString> names for nodes
 **/
QSet<QString> QNode::getNodes()
{
    QSet<QString> nodes;
    ros::V_string node_info;
    ros::master::getNodes(node_info);

    for(auto node: node_info)
        nodes.insert(node.c_str());

    return nodes;
}

/** @brief update topic list, based on retrieved info
 *  @returns QSet<QString> of new-untracked topics
 **/
QSet<QString> QNode::updateTopics()
{
    QSet<QString> current_topics(getTopics());

    current_topics.subtract(m_topic_list);

    return current_topics;
}


/** @brief add new_topics to viewtable
 **/
void QNode::updateTopicTable()
{
    QSet<QString> new_topics(updateTopics());
    QPair<QString, QString> topic_msg;
    //last->setText("lol");
    // if there are new topics add them to tableview
    if(!new_topics.empty())
    {
        for(auto topic_str: new_topics.values())
        {
            QVariant new_row(topic_str);

            m_topic_list.insert(topic_str);
            //auto last = m_topic_widgets.end();

            //last->setText("topic_str");


            qDebug() << "New_topic: " << topic_str;
            topic_msg.first = "topic_widget";
            topic_msg.second = "really";
            Q_EMIT loggingUpdated(topic_msg);
        }
    }

     // used to readjust the scrollbar
}

}  // namespace rqt_state
