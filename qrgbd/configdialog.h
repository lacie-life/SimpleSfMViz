#ifndef CONFIGDIALOG_H
#define CONFIGDIALOG_H

#include <QDialog>
#include <QString>

namespace Ui
{
    class ConfigDialog;
}

class ConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConfigDialog(QWidget *parent = nullptr);
    ~ConfigDialog();

    void connection();

    void display();

    bool isSetted();

    void closeEvent(QCloseEvent *e) override;

public:
    Ui::ConfigDialog *ui;

    QString _seqPath;
    QString _colorImgPath;
    QString _depthImgPath;

    QString _vocPath;
    QString _settingPath;
    QString _assoPath;

    QString _classes;
    QString _modelConfig;
    QString _modelWeights;

    QString _temp_seqPath;
    QString _temp_colorImgPath;
    QString _temp_depthImgPath;

    QString _temp_vocPath;
    QString _temp_settingPath;
    QString _temp_assoPath;

    QString _temp_classes;
    QString _temp_modelConfig;
    QString _temp_modelWeights;
};

#endif // CONFIGDIALOG_H
