/**
 * @file mainwindow.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "tcpclient01.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void startaction0();
    void startaction1();
    void startaction2();
    void startaction3();
    void startaction4();

private:
    Ui::MainWindow *ui;
    TcpClient01 tcpClient01;
};
#endif // MAINWINDOW_H
