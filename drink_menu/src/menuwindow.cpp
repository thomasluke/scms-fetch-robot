#include "menuwindow.h"
#include "./ui_menuwindow.h"

MenuWindow::MenuWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MenuWindow)
{
    std::random_device rd;
    nd_generator = new std::default_random_engine(rd()); // seed the generator
    random_drink = new std::uniform_int_distribution<>(1, 6);

    ui->setupUi(this);
}

MenuWindow::~MenuWindow()
{
    delete ui;
    delete nd_generator;
    delete random_drink;
}

void MenuWindow::on_ginTonic_clicked()
{
    ui->drinkChoice->setText("Gin and Tonic");
}

void MenuWindow::on_vodkaLemonade_clicked()
{
    ui->drinkChoice->setText("Vodka Lemonade");
}

void MenuWindow::on_vodkaMartini_clicked()
{
    ui->drinkChoice->setText("Vodka Martini");
}

void MenuWindow::on_ginMartini_clicked()
{
    ui->drinkChoice->setText("Gin Martini");
}

void MenuWindow::on_negroni_clicked()
{
    ui->drinkChoice->setText("Negroni");
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
