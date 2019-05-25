/**
 * @file /include/rqt_state/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rqt_state_QNODE_HPP_
#define rqt_state_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>




/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rqt_state {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();


	QStringListModel* loggingModel() { return &logging_model; }
        void updateTopicTable();

Q_SIGNALS:
	void loggingUpdated();
        void rosShutdown();

// private methods
private:
        QSet<QString> getTopics();
        QSet<QString> getNodes();

        QSet<QString> updateTopics();

        void topicCallback(const topic_tools::ShapeShifter::ConstPtr &msg,
                           uint64_t &msg_time);

private:

	int init_argc;
	char** init_argv;

        QStringListModel logging_model;
        QSet<QString> m_topic_list;
        QSet<QString> m_node_list;

        std::vector<ros::Subscriber> m_topic_subs;
};

}  // namespace rqt_state

#endif /* rqt_state_QNODE_HPP_ */
