#include <QGuiApplication>
#include "openGL/QPointCloudViewer.h"
#include "openGL/QPointCloudUnderlay.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    QPointCloudViewer w;
    w.resize(600, 600);
    w.show();

    return app.exec();
}
