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

    drink_pub = n.advertise<drink_menu::drink>("drink_selection", 1000);
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
    publishDrink("Gin and Tonic", std::vector<std::string>{"gin"});
}

void MenuWindow::on_vodkaLemonade_clicked()
{
    ui->drinkChoice->setText("Vodka Lemonade");
    publishDrink("Vodka Lemonade", std::vector<std::string>{"vodka"});
}

void MenuWindow::on_vodkaMartini_clicked()
{
    ui->drinkChoice->setText("Vodka Martini");
    publishDrink("Vodka Martini", std::vector<std::string>{"vodka", "vermouth"});
}

void MenuWindow::on_ginMartini_clicked()
{
    ui->drinkChoice->setText("Gin Martini");
    publishDrink("Gin Martini", std::vector<std::string>{"gin", "vermouth"});
}

void MenuWindow::on_negroni_clicked()
{
    ui->drinkChoice->setText("Negroni");
    publishDrink("Negroni", std::vector<std::string>{"gin", "vermouth", "campari"});
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

void MenuWindow::publishDrink(std::string drink, std::vector<std::string> ingredients)
{
    if (ros::ok())
    {
        drink_menu::drink msg;
        msg.drink = drink;
        msg.ingredients = ingredients;

        ROS_INFO("%s", msg.drink.c_str());
        drink_pub.publish(msg);
        ros::spinOnce();
    }
}