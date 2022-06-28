#include "embed.h"
#include <iostream>

QWidget *cvEmbedWindow(const std::string &winName)
{
    cv::namedWindow(winName.c_str(), CV_WINDOW_FREERATIO);
    // get gtk widget
    GtkWidget *cvWin = (GtkWidget *)cvGetWindowHandle(winName.c_str());
    // get the window's parent
    auto paWin = gtk_widget_get_parent_window(cvWin);
    // get parent's id
    XID id = GDK_WINDOW_XID(paWin);
    // create a QWindow from the id
    QWindow *qWin = QWindow::fromWinId(WId(id));
    // create QWidget from the QWindow
    QWidget *widget = QWidget::createWindowContainer(qWin);
    return widget;
}
