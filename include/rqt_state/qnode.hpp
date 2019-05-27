#ifndef rqt_state_QNODE_HPP_
#define rqt_state_QNODE_HPP_
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Int64.h>
#include "enum_msg_type.h"
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QTextBrowser>

namespace rqt_state {

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();
    void addTopicWidget(const QString &object_name, const QString &topic_name, msg_type msg_type);

Q_SIGNALS:
    void loggingUpdated(QPair<QString, QString> msg);
    void rosShutdown();

private:
    void int64TopicCallback(const std_msgs::Int64::ConstPtr& msg,
                            const QString &object_name);

private:
    int init_argc;
    char** init_argv;

    boost::shared_ptr<ros::NodeHandle> m_nh;

    QSet<QString> m_topic_widgets;
    std::vector<ros::Subscriber> m_topic_subs;
};

}  // namespace rqt_state

#endif /* rqt_state_QNODE_HPP_ */
