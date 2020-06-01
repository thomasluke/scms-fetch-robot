#include "menuwindow.h"
#include "./ui_menuwindow.h"
#include <string>

// MenuWindow::MenuWindow(QWidget *parent)
//     : QMainWindow(parent), ui(new Ui::MenuWindow)
// {
//     std::random_device rd;
//     nd_generator = new std::default_random_engine(rd()); // seed the generator
//     random_drink = new std::uniform_int_distribution<>(1, 6);

//     ui->setupUi(this);

//     drink_pub
// }

MenuWindow::MenuWindow(ros::NodeHandle n, QWidget *parent) : QMainWindow(parent), ui(new Ui::MenuWindow)
{
    std::random_device rd;
    nd_generator = new std::default_random_engine(rd()); // seed the generator
    random_drink = new std::uniform_int_distribution<>(1, 6);

    ui->setupUi(this);

    drink_pub = n.advertise<std_msgs::String>("drink_selection", 1000);
}

MenuWindow::~MenuWindow()
{
    delete ui;
    delete nd_generator;
    delete random_drink;
}

void MenuWindow::lockButtons()
{
    QList<QPushButton *> buttons = ui->centralwidget->findChildren<QPushButton *>();
    for (int i = 0; i < buttons.size(); ++i)
    {
        buttons.at(i)->setEnabled(false);
    }
}

void MenuWindow::unlockButtons()
{
    QList<QPushButton *> buttons = ui->centralwidget->findChildren<QPushButton *>();
    for (int i = 0; i < buttons.size(); ++i)
    {
        buttons.at(i)->setEnabled(false);
    }
}

void MenuWindow::on_ginTonic_clicked()
{
    ui->drinkChoice->setText("Gin and Tonic");
    publishDrink("Gin and Tonic");
}

void MenuWindow::on_vodkaLemonade_clicked()
{
    ui->drinkChoice->setText("Vodka Lemonade");
    publishDrink("Vodka Lemonade");
}

void MenuWindow::on_vodkaMartini_clicked()
{
    ui->drinkChoice->setText("Vodka Martini");
    publishDrink("Vodka Martini");
}

void MenuWindow::on_ginMartini_clicked()
{
    ui->drinkChoice->setText("Gin Martini");
    publishDrink("Gin Martini");
}

void MenuWindow::on_negroni_clicked()
{
    ui->drinkChoice->setText("Negroni");
    publishDrink("Negroni");
}

void MenuWindow::on_surpriseMe_clicked()
{
    int random = (*random_drink)(*nd_generator);
    switch (random)
    {
    default:
    case 1:
        on_ginTonic_clicked();
        break;
    case 2:
        on_vodkaLemonade_clicked();
        break;
    case 3:
        on_vodkaMartini_clicked();
        break;
    case 4:
        on_ginMartini_clicked();
        break;
    case 5:
        on_negroni_clicked();
        break;
    }
}

void MenuWindow::publishDrink(std::string drink)
{
    if (ros::ok())
    {
        std_msgs::String msg;
        msg.data = drink;

        ROS_INFO("%s", msg.data.c_str());
        drink_pub.publish(msg);
        ros::spinOnce();
    }
}