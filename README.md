# rqt_state
QT package, enabling to define topic boxes and assign topic names to them

## Usage
1. Add Qt Text Edit Input Widget to the frame.
2. Add dynamic property with the string field "topic_name" to that widget.
3. Go to *main_window.cpp* and add following lines to `MainWindow::MainWindow` constructor
```c++
m_qt_name_to_object[ui.topic_viewer->objectName()] = ui.topic_viewer;
m_qnode.addTopicWidget(ui.topic_viewer->objectName(), ui.topic_viewer->property("topic_name").toString(), STATS_DROPPED_MSGS);
```
4. Change *ui.topic_viewer* to the name of the object on ui.
5. If the topic have msg_type other than Statistics or Int64, do the following steps:

    1. Add msg_enum in *enum_msg_type.h*
    2. Add callback prototype to *qnode.hpp*
    3. Add callback implementation to *qnode.cpp*, with **mandatory** `Q_EMIT loggingUpdated(qt_msg)`, where qt_msg is the `QPair<Qstring, QString>` of qt object_name on ui and string, that is going to be displayed in box.
    4. Add `ros::subscriber` to the `QNode::addTopicwidget` with the pattern presented in INT_64 and STATS_* msg_types, but use your own callback and mgs_type. If you want to monitor topic, then use the pattern in INT_64. If you want to monitor statistics on topic, then use the pattern in STATS* msg_types.

## Few comments and known bugs
1. To enable statistics write `rosparam set /enable_statistics true` in terminal and only after that launch ondes
2. The information about topic stats will **only** be published, if there is some node is subscribed to the topic. Seems, we cannot escape that, because this is how ROS works and it is impossible to get statistics from dead leafs(topics with no subscribers).