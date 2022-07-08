#ifndef QCONFIGDIALOG_H
#define QCONFIGDIALOG_H

#include <QDialog>
#include <QObject>
#include <QString>

namespace Ui
{
    class QConfigDialog;
}

class QConfigDialog : public QDialog
{
    Q_OBJECT
public:
    QConfigDialog(QWidget *parent = nullptr);
    ~QConfigDialog();

    void connection();

    void display();

    bool isSetted();

    void closeEvent(QCloseEvent *e) override;

public:
    Ui::QConfigDialog *ui;

    QString m_seqPath;
    QString m_colorImgPath;
    QString m_depthImgPath;

    QString m_vocPath;
    QString m_settingPath;
    QString m_assoPath_d;
    QString m_assoPath_rgb;

    QString m_classes;
    QString m_modelConfig;
    QString m_modelWeights;

    QString m_temp_seqPath;
    QString m_temp_colorImgPath;
    QString m_temp_depthImgPath;

    QString m_temp_vocPath;
    QString m_temp_settingPath;
    QString m_temp_assoPath_d;
    QString m_temp_assoPath_rgb;

    QString m_temp_classes;
    QString m_temp_modelConfig;
    QString m_temp_modelWeights;
};

#endif // QCONFIGDIALOG_H
