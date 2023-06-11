/**
 * @file mainwindow.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , tcpClient01(0, USE_CRC)
{
    ui->setupUi(this);
    connect(ui->start0, &QPushButton::clicked, this, &MainWindow::startaction0);
    connect(ui->start1, &QPushButton::clicked, this, &MainWindow::startaction1);
    connect(ui->start2, &QPushButton::clicked, this, &MainWindow::startaction2);
    connect(ui->start3, &QPushButton::clicked, this, &MainWindow::startaction3);
    connect(ui->start4, &QPushButton::clicked, this, &MainWindow::startaction4);

    tcpClient01.start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::startaction0()
{
    QString txt = ui->start0->text();
    if (!txt.toStdString().compare("Start measurementLoop0"))
    {
        ui->start0->setText("Started measurementLoop0");
        tcpClient01.startMeasurementLoop0();
    }
    else
    {
        ui->start0->setText("Start measurementLoop0");
        tcpClient01.stopMeasurementLoop0();
    }
}

void MainWindow::startaction1()
{
    QString txt = ui->start1->text();
    if (!txt.toStdString().compare("Start measurementLoop1"))
    {
        ui->start1->setText("Started measurementLoop1");
        tcpClient01.startMeasurementLoop1();
    }
    else
    {
        ui->start1->setText("Start measurementLoop1");
        tcpClient01.stopMeasurementLoop1();
    }
}

void MainWindow::startaction2()
{
    QString txt = ui->start2->text();
    if (!txt.toStdString().compare("Start measurementLoop2"))
    {
        ui->start2->setText("Started measurementLoop2");
        tcpClient01.startMeasurementLoop2();
    }
    else
    {
        ui->start2->setText("Start measurementLoop2");
        tcpClient01.stopMeasurementLoop2();
    }
}

void MainWindow::startaction3()
{
    QString txt = ui->start3->text();
    if (!txt.toStdString().compare("Start measurementLoop3"))
    {
        ui->start3->setText("Started measurementLoop3");
        tcpClient01.startMeasurementLoop3();
    }
    else
    {
        ui->start3->setText("Start measurementLoop3");
        tcpClient01.stopMeasurementLoop3();
    }
}

void MainWindow::startaction4()
{
    QString txt = ui->start4->text();
    if (!txt.toStdString().compare("Start measurementLoop4"))
    {
        ui->start4->setText("Started measurementLoop4");
        tcpClient01.startMeasurementLoop4();
    }
    else
    {
        ui->start4->setText("Start measurementLoop4");
        tcpClient01.stopMeasurementLoop4();
    }
}
