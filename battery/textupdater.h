#ifndef TEXTUPDATER_H
#define TEXTUPDATER_H

#include <QObject>
#include <QWidget>
#include <QLineEdit>

class TextUpdater
{
public:
    TextUpdater(QLineEdit* lineEdit);
public slots:
    void updateText();
};

#endif // TEXTUPDATER_H
