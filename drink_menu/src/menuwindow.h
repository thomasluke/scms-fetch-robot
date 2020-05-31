#ifndef MENUWINDOW_H
#define MENUWINDOW_H

#include <QMainWindow>
#include <random>

QT_BEGIN_NAMESPACE
namespace Ui { class MenuWindow; }
QT_END_NAMESPACE

class MenuWindow : public QMainWindow
{
    Q_OBJECT

public:
    MenuWindow(QWidget *parent = nullptr);
    ~MenuWindow();

private slots:
    void on_ginTonic_clicked();

    void on_vodkaLemonade_clicked();

    void on_vodkaMartini_clicked();

    void on_ginMartini_clicked();

    void on_negroni_clicked();

    void on_surpriseMe_clicked();

private:
    Ui::MenuWindow *ui;
    std::default_random_engine  *nd_generator;
    std::uniform_int_distribution<> *random_drink;
};
#endif // MENUWINDOW_H
