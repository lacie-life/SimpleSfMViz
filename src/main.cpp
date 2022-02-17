#include <QGuiApplication>
#include "QPointCloudViewer.h"
#include "QPointCloudUnderlay.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    QPointCloudUnderlay w;
    w.resize(600, 600);
    w.show();

    return app.exec();
}
