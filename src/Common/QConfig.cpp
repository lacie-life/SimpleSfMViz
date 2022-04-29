#include "QConfig.h"

QConfig::QConfig(QObject *parent)
    : QObject{parent},
      currentModel(AppEnums::DETECT_MODEL::YOLO)
{

}

AppEnums::DETECT_MODEL QConfig::modelType() const
{
    return currentModel;
}

void QConfig::setModelType(AppEnums::DETECT_MODEL model)
{
    currentModel = model;

    emit modelTypeChanged(currentModel);
}
