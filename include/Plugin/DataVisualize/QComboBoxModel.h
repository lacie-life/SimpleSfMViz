#ifndef QCOMBOBOXMODEL_H
#define QCOMBOBOXMODEL_H

#include <QObject>
#include <QAbstractListModel>
#include <QVariant>
#include <QVector>
#include <QDebug>

// Ref: https://stackoverflow.com/questions/64236237/how-to-add-an-extra-item-to-a-qml-combobox-which-is-not-in-the-model

class QComboBoxModel : public QAbstractListModel
{
    Q_OBJECT
public:
    explicit QComboBoxModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

signals:

private:
    QVector<QString> mData;
};

#endif // QCOMBOBOXMODEL_H
