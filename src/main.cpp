#include "QPointCloudViewer.h"
#include "QPointCloudUnderlay.h"

#include <QGuiApplication>

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    QPointCloudViewer view;
    view.show();
    view.resize(600, 600);

    return app.exec();
}
