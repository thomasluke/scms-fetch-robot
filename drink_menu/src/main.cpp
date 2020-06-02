#include "menuwindow.h"
#include <QApplication>
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drink_menu");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    // ros::spin();
    // ros::shutdown();

    QApplication a(argc, argv);
    MenuWindow w(n);
    w.show();
    return a.exec();
}
