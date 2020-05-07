#include "menugui.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MenuGUI w;
    w.show();
    return a.exec();
}
