#include <QtGui>
#include <QApplication>
#include "../include/open_manipulator_x_gui/main_window.hpp"
#include "open_manipulator_x_gui/qnode.hpp"

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    open_manipulator_x_gui::MainWindow w(argc, argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    int result = app.exec();
    return result;
}
