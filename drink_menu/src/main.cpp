#include "menuwindow.h"
#include <QApplication>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MenuWindow w;
    w.show();
    return a.exec();
}
