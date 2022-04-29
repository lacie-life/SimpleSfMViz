#include "DataVisualize/QComboBoxModel.h"
#include "AppEnums.h"

QComboBoxModel::QComboBoxModel(QObject *parent)
    : QAbstractListModel(parent)
{
    for(int i = 0; i < AppEnums::DETECT_MODEL::MODEL_MAX; i++){
        mData.append(AppEnums::MODEL_ZOO[i]);
    }
}

int QComboBoxModel::rowCount(const QModelIndex &parent) const
{

}

QVariant QComboBoxModel::data(const QModelIndex &index, int role) const
{

}
