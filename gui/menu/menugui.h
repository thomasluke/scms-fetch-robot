#ifndef MENUGUI_H
#define MENUGUI_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MenuGUI; }
QT_END_NAMESPACE

class MenuGUI : public QMainWindow
{
    Q_OBJECT

public:
    MenuGUI(QWidget *parent = nullptr);
    ~MenuGUI();

private:
    Ui::MenuGUI *ui;
};
#endif // MENUGUI_H
