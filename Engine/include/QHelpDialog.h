#ifndef QHELPDIALOG_H
#define QHELPDIALOG_H

#include <QDialog>

namespace Ui {
class QHelpDialog;
}

class QHelpDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QHelpDialog(QWidget *parent = nullptr);
    ~QHelpDialog();

private:
    Ui::QHelpDialog *ui;
};

#endif // QHELPDIALOG_H
