#include <QtGui>
#include <QApplication>
#include "../include/rqt_state/main_window.hpp"

int main(int argc, char **argv) {

    QApplication app(argc, argv);
    rqt_state::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    return result;
}
