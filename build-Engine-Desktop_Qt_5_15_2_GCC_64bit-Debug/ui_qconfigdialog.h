/********************************************************************************
** Form generated from reading UI file 'qconfigdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QCONFIGDIALOG_H
#define UI_QCONFIGDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>

QT_BEGIN_NAMESPACE

class Ui_QConfigDialog
{
public:
    QGridLayout *gridLayout_12;
    QGroupBox *groupBox_9;
    QGridLayout *gridLayout_8;
    QHBoxLayout *horizontalLayout;
    QLabel *label_7;
    QLineEdit *lineEdit_class;
    QPushButton *load_classes;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_8;
    QLineEdit *lineEdit_modelConfig;
    QPushButton *load_modelConfig;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_9;
    QLineEdit *lineEdit_modelWeight;
    QPushButton *load_modelWeight;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *btn_ok;
    QSpacerItem *horizontalSpacer;
    QPushButton *btn_cancel;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_4;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QLineEdit *lineEdit_voc;
    QPushButton *load_voc;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_6;
    QLineEdit *lineEdit_set;
    QPushButton *load_set;
    QSpacerItem *horizontalSpacer_2;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label;
    QLineEdit *lineEdit_seq;
    QPushButton *load_seq;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_4;
    QLineEdit *lineEdit_asso;
    QHBoxLayout *horizontalLayout_10;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_2;
    QLineEdit *lineEdit_color;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_3;
    QLineEdit *lineEdit_depth;

    void setupUi(QDialog *QConfigDialog)
    {
        if (QConfigDialog->objectName().isEmpty())
            QConfigDialog->setObjectName(QString::fromUtf8("QConfigDialog"));
        QConfigDialog->resize(779, 493);
        gridLayout_12 = new QGridLayout(QConfigDialog);
        gridLayout_12->setObjectName(QString::fromUtf8("gridLayout_12"));
        groupBox_9 = new QGroupBox(QConfigDialog);
        groupBox_9->setObjectName(QString::fromUtf8("groupBox_9"));
        QFont font;
        font.setFamily(QString::fromUtf8("Ubuntu Mono"));
        font.setPointSize(12);
        font.setItalic(true);
        groupBox_9->setFont(font);
        gridLayout_8 = new QGridLayout(groupBox_9);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_7 = new QLabel(groupBox_9);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setMinimumSize(QSize(130, 0));
        label_7->setFont(font);
        label_7->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout->addWidget(label_7);

        lineEdit_class = new QLineEdit(groupBox_9);
        lineEdit_class->setObjectName(QString::fromUtf8("lineEdit_class"));
        lineEdit_class->setMinimumSize(QSize(0, 30));
        lineEdit_class->setFont(font);

        horizontalLayout->addWidget(lineEdit_class);

        load_classes = new QPushButton(groupBox_9);
        load_classes->setObjectName(QString::fromUtf8("load_classes"));
        load_classes->setMinimumSize(QSize(80, 30));
        load_classes->setMaximumSize(QSize(200, 16777215));
        load_classes->setFont(font);

        horizontalLayout->addWidget(load_classes);


        gridLayout_8->addLayout(horizontalLayout, 0, 0, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_8 = new QLabel(groupBox_9);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setMinimumSize(QSize(130, 0));
        label_8->setFont(font);
        label_8->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_2->addWidget(label_8);

        lineEdit_modelConfig = new QLineEdit(groupBox_9);
        lineEdit_modelConfig->setObjectName(QString::fromUtf8("lineEdit_modelConfig"));
        lineEdit_modelConfig->setMinimumSize(QSize(0, 30));
        lineEdit_modelConfig->setFont(font);

        horizontalLayout_2->addWidget(lineEdit_modelConfig);

        load_modelConfig = new QPushButton(groupBox_9);
        load_modelConfig->setObjectName(QString::fromUtf8("load_modelConfig"));
        load_modelConfig->setMinimumSize(QSize(80, 30));
        load_modelConfig->setMaximumSize(QSize(200, 16777215));
        load_modelConfig->setFont(font);

        horizontalLayout_2->addWidget(load_modelConfig);


        gridLayout_8->addLayout(horizontalLayout_2, 1, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_9 = new QLabel(groupBox_9);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setMinimumSize(QSize(130, 0));
        label_9->setFont(font);
        label_9->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_3->addWidget(label_9);

        lineEdit_modelWeight = new QLineEdit(groupBox_9);
        lineEdit_modelWeight->setObjectName(QString::fromUtf8("lineEdit_modelWeight"));
        lineEdit_modelWeight->setMinimumSize(QSize(0, 30));
        lineEdit_modelWeight->setFont(font);

        horizontalLayout_3->addWidget(lineEdit_modelWeight);

        load_modelWeight = new QPushButton(groupBox_9);
        load_modelWeight->setObjectName(QString::fromUtf8("load_modelWeight"));
        load_modelWeight->setMinimumSize(QSize(80, 30));
        load_modelWeight->setMaximumSize(QSize(200, 16777215));
        load_modelWeight->setFont(font);

        horizontalLayout_3->addWidget(load_modelWeight);


        gridLayout_8->addLayout(horizontalLayout_3, 2, 0, 1, 1);


        gridLayout_12->addWidget(groupBox_9, 2, 0, 1, 5);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout_12->addItem(horizontalSpacer_3, 3, 2, 1, 1);

        btn_ok = new QPushButton(QConfigDialog);
        btn_ok->setObjectName(QString::fromUtf8("btn_ok"));
        btn_ok->setMinimumSize(QSize(100, 30));
        btn_ok->setMaximumSize(QSize(200, 16777215));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Ubuntu Mono"));
        font1.setPointSize(14);
        font1.setItalic(true);
        btn_ok->setFont(font1);

        gridLayout_12->addWidget(btn_ok, 3, 3, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_12->addItem(horizontalSpacer, 3, 0, 1, 1);

        btn_cancel = new QPushButton(QConfigDialog);
        btn_cancel->setObjectName(QString::fromUtf8("btn_cancel"));
        btn_cancel->setMinimumSize(QSize(100, 30));
        btn_cancel->setMaximumSize(QSize(200, 16777215));
        btn_cancel->setFont(font1);

        gridLayout_12->addWidget(btn_cancel, 3, 1, 1, 1);

        groupBox_3 = new QGroupBox(QConfigDialog);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setFont(font);
        gridLayout_4 = new QGridLayout(groupBox_3);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_5 = new QLabel(groupBox_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setMinimumSize(QSize(130, 0));
        label_5->setFont(font);
        label_5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_5->addWidget(label_5);

        lineEdit_voc = new QLineEdit(groupBox_3);
        lineEdit_voc->setObjectName(QString::fromUtf8("lineEdit_voc"));
        lineEdit_voc->setMinimumSize(QSize(0, 30));
        lineEdit_voc->setFont(font);

        horizontalLayout_5->addWidget(lineEdit_voc);

        load_voc = new QPushButton(groupBox_3);
        load_voc->setObjectName(QString::fromUtf8("load_voc"));
        load_voc->setMinimumSize(QSize(80, 30));
        load_voc->setMaximumSize(QSize(200, 16777215));
        load_voc->setFont(font);

        horizontalLayout_5->addWidget(load_voc);


        gridLayout_4->addLayout(horizontalLayout_5, 0, 0, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_6 = new QLabel(groupBox_3);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setMinimumSize(QSize(130, 0));
        label_6->setFont(font);
        label_6->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_4->addWidget(label_6);

        lineEdit_set = new QLineEdit(groupBox_3);
        lineEdit_set->setObjectName(QString::fromUtf8("lineEdit_set"));
        lineEdit_set->setMinimumSize(QSize(0, 30));
        lineEdit_set->setFont(font);

        horizontalLayout_4->addWidget(lineEdit_set);

        load_set = new QPushButton(groupBox_3);
        load_set->setObjectName(QString::fromUtf8("load_set"));
        load_set->setMinimumSize(QSize(80, 30));
        load_set->setMaximumSize(QSize(200, 16777215));
        load_set->setFont(font);

        horizontalLayout_4->addWidget(load_set);


        gridLayout_4->addLayout(horizontalLayout_4, 1, 0, 1, 1);


        gridLayout_12->addWidget(groupBox_3, 1, 0, 1, 5);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout_12->addItem(horizontalSpacer_2, 3, 4, 1, 1);

        groupBox = new QGroupBox(QConfigDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setFont(font);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMinimumSize(QSize(130, 0));
        label->setFont(font);
        label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_9->addWidget(label);

        lineEdit_seq = new QLineEdit(groupBox);
        lineEdit_seq->setObjectName(QString::fromUtf8("lineEdit_seq"));
        lineEdit_seq->setMinimumSize(QSize(0, 30));
        lineEdit_seq->setFont(font);

        horizontalLayout_9->addWidget(lineEdit_seq);

        load_seq = new QPushButton(groupBox);
        load_seq->setObjectName(QString::fromUtf8("load_seq"));
        load_seq->setMinimumSize(QSize(80, 30));
        load_seq->setMaximumSize(QSize(200, 16777215));
        load_seq->setFont(font);

        horizontalLayout_9->addWidget(load_seq);


        gridLayout_2->addLayout(horizontalLayout_9, 0, 0, 1, 1);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMinimumSize(QSize(130, 0));
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_6->addWidget(label_4);

        lineEdit_asso = new QLineEdit(groupBox);
        lineEdit_asso->setObjectName(QString::fromUtf8("lineEdit_asso"));
        lineEdit_asso->setMinimumSize(QSize(0, 30));
        lineEdit_asso->setFont(font);

        horizontalLayout_6->addWidget(lineEdit_asso);


        gridLayout_2->addLayout(horizontalLayout_6, 2, 0, 1, 1);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setMinimumSize(QSize(130, 0));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_7->addWidget(label_2);

        lineEdit_color = new QLineEdit(groupBox);
        lineEdit_color->setObjectName(QString::fromUtf8("lineEdit_color"));
        lineEdit_color->setMinimumSize(QSize(0, 30));
        lineEdit_color->setFont(font);

        horizontalLayout_7->addWidget(lineEdit_color);


        horizontalLayout_10->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setMinimumSize(QSize(130, 0));
        label_3->setFont(font);
        label_3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_8->addWidget(label_3);

        lineEdit_depth = new QLineEdit(groupBox);
        lineEdit_depth->setObjectName(QString::fromUtf8("lineEdit_depth"));
        lineEdit_depth->setMinimumSize(QSize(0, 30));
        lineEdit_depth->setFont(font);

        horizontalLayout_8->addWidget(lineEdit_depth);


        horizontalLayout_10->addLayout(horizontalLayout_8);


        gridLayout_2->addLayout(horizontalLayout_10, 1, 0, 1, 1);


        gridLayout_12->addWidget(groupBox, 0, 0, 1, 5);


        retranslateUi(QConfigDialog);

        QMetaObject::connectSlotsByName(QConfigDialog);
    } // setupUi

    void retranslateUi(QDialog *QConfigDialog)
    {
        QConfigDialog->setWindowTitle(QCoreApplication::translate("QConfigDialog", "Dialog", nullptr));
        groupBox_9->setTitle(QCoreApplication::translate("QConfigDialog", "Recongnizer", nullptr));
        label_7->setText(QCoreApplication::translate("QConfigDialog", "Classes", nullptr));
        load_classes->setText(QCoreApplication::translate("QConfigDialog", "load", nullptr));
        label_8->setText(QCoreApplication::translate("QConfigDialog", "Model Config", nullptr));
        load_modelConfig->setText(QCoreApplication::translate("QConfigDialog", "load", nullptr));
        label_9->setText(QCoreApplication::translate("QConfigDialog", "Model Weights", nullptr));
        load_modelWeight->setText(QCoreApplication::translate("QConfigDialog", "load", nullptr));
        btn_ok->setText(QCoreApplication::translate("QConfigDialog", "Ok", nullptr));
        btn_cancel->setText(QCoreApplication::translate("QConfigDialog", "Cancel", nullptr));
        groupBox_3->setTitle(QCoreApplication::translate("QConfigDialog", "Data Configure", nullptr));
        label_5->setText(QCoreApplication::translate("QConfigDialog", "Vocabulary Path", nullptr));
        load_voc->setText(QCoreApplication::translate("QConfigDialog", "load", nullptr));
        label_6->setText(QCoreApplication::translate("QConfigDialog", "Settings Path", nullptr));
        load_set->setText(QCoreApplication::translate("QConfigDialog", "load", nullptr));
        groupBox->setTitle(QCoreApplication::translate("QConfigDialog", "Data Set", nullptr));
        label->setText(QCoreApplication::translate("QConfigDialog", "Sequence Path", nullptr));
        load_seq->setText(QCoreApplication::translate("QConfigDialog", "load", nullptr));
        label_4->setText(QCoreApplication::translate("QConfigDialog", "Association Path", nullptr));
        label_2->setText(QCoreApplication::translate("QConfigDialog", "Color Image Path", nullptr));
        label_3->setText(QCoreApplication::translate("QConfigDialog", "Depth Image Path", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QConfigDialog: public Ui_QConfigDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QCONFIGDIALOG_H
