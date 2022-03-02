#include "SfM/QSfM.h"


QSfM::QSfM(QObject *parent)
    : QObject(parent)
{

}

void QSfM::init(QString imgFolder)
{

}

void QSfM::run()
{

}

QString QSfM::imgFolder() const
{
    return m_imgFolder;
}

QString QSfM::pointCloudPath() const
{
    return m_pointCloudPath;
}

void QSfM::setImgFolder(QString path)
{
    m_imgFolder = path;

    emit imgFolderChanged(path);
}

void QSfM::setPointCloudPath(QString path)
{
    m_pointCloudPath = path;

    emit pointCloudPathChanged(path);
}
