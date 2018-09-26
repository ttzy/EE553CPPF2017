#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    /* Settings for progressbar */
    ui->progressBar->setMaximum(101);
    ui->progressBar->setMinimum(0);
    ui->progressBar->setValue(0);
    ui->progressBar1->setMaximum(101);
    ui->progressBar1->setMinimum(0);
    ui->progressBar1->setValue(0);
    ui->progressBar2->setMaximum(101);
    ui->progressBar2->setMinimum(0);
    ui->progressBar2->setValue(0);
    ui->progressBar3->setMaximum(101);
    ui->progressBar3->setMinimum(0);
    ui->progressBar3->setValue(0);
    /* Settings for LineEdit*/
    ui->voltage1->setText("0V");
    ui->voltage2->setText("0V");
    ui->voltage3->setText("0V");
    ui->voltage4->setText("0V");
    /* Add a tab in the TabWidget */

    voltage = 0;
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(val_to_voltage()));
    connect(timer, SIGNAL(timeout()), this, SLOT(progress()));
    timer->start(1000);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::progress()
{
    ui->progressBar->setValue(voltage);
    ui->progressBar1->setValue(voltage);
    ui->progressBar2->setValue(voltage);
    ui->progressBar3->setValue(voltage);
}

void MainWindow::val_to_voltage()
{
    ui->voltage1->setText(QString::number(voltage) + "V");
    ui->voltage2->setText(QString::number(voltage) + "V");
    ui->voltage3->setText(QString::number(voltage) + "V");
    ui->voltage4->setText(QString::number(voltage) + "V");
    voltage = voltage + 1;
}

void MainWindow::on_tabWidget_tabCloseRequested(int index)
{
    ui->tabWidget->removeTab(index);
}
