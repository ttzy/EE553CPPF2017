#include "textupdater.h"

TextUpdater::TextUpdater(QLineEdit* edit)
       :QObject(lineEdit), lineEdit(edit)
{
    QTimer* timer = new QTimer(this);
    timer->setSingleShot(false);
    timer->setInterval(10 * 1000); // 10 seconds
    connect(timer, SIGNAL(timeout()), this, SLOT(updateText()));
}

void TextUpdater::updateText()
{
  // Set the text to whatever you want. This is just to show it updating
  lineEdit->setText(QTime::currentTime().toString());
}
