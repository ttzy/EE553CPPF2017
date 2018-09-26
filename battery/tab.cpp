#include "tab.h"
#include "ui_tab.h"

Tab::Tab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab)
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

    voltage = 0;
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(val_to_voltage()));
    connect(timer, SIGNAL(timeout()), this, SLOT(progress()));
    timer->start(1000);
}

Tab::~Tab()
{
    delete ui;
}
