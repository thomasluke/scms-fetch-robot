#include "menugui.h"
#include "./ui_menugui.h"

MenuGUI::MenuGUI(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MenuGUI)
{
    ui->setupUi(this);
}

MenuGUI::~MenuGUI()
{
    delete ui;
}

