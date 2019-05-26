/**
 * @file /include/rqt_state/main_window.hpp
 *
 * @brief Qt based gui for rqt_state.
 *
 * @date November 2010
 **/
#ifndef rqt_state_MAIN_WINDOW_H
#define rqt_state_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"


namespace rqt_state {

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

public Q_SLOTS:
        void updateTopicBox(QPair<QString, QString> msg);

private:
	Ui::MainWindowDesign ui;
        QMap<QString, QTextBrowser*> topic_name_widget;
	QNode qnode;

};

}  // namespace rqt_state

#endif // rqt_state_MAIN_WINDOW_H
