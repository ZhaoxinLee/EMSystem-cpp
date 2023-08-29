#include "mainwindow.h"
#include <QApplication>
#include "gamepadmonitor.h"

// MainWindow* window;

int main(int argc, char *argv[])
{
    // Default lines of code to launch Qt GUI
    QApplication a(argc, argv);
    MainWindow window;
    window.show();

    // Changed new window object to pointer equivalent
//    window = new MainWindow();
//    window->show();
    // NOTE: This previous method of calling new (Object) and
    // not calling delete (Object) can appearently cause memory leaks.
    // Reverted the code back to not using pointers here to successfully execute
    // Destroyer code blocks, close connections properly, and avoid mem leaks.

    // Run the system code from mainwindow.cpp
    return a.exec();
}
