#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
public slots:
    void val_to_voltage();
    void progress();

private slots:
    void on_tabWidget_tabCloseRequested(int index);

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    int voltage;
};

#endif // MAINWINDOW_H
