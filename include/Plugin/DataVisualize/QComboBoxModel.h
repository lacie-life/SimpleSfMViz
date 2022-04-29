#ifndef QCOMBOBOXMODEL_H
#define QCOMBOBOXMODEL_H

#include <QObject>
#include <QStringListModel>
#include <QVariant>
#include <QDebug>
#include "AppEnums.h"
#include "AppConstant.h"

// https://stackoverflow.com/questions/18616497/how-to-use-models-with-qml

//class QComboBoxModel : public QObject
//{
//    Q_OBJECT
//    QStringListModel * m_model;
//public:
//    QComboBoxModel(QStringListModel * model) : m_model(model) {}
//    Q_INVOKABLE void generate() {
//        QStringList list;
//        for (int i = 1; i <= AppEnums::MODEL_ZOO.size(); ++i) {
//            list << AppEnums::MODEL_ZOO[i];
//        }
//        m_model->setStringList(list);
//    }
//};

#endif // QCOMBOBOXMODEL_H
