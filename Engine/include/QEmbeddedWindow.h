#ifndef QEMBEDDEDWINDOW_H
#define QEMBEDDEDWINDOW_H

#include "ui_mainwindow.h"
#undef signals
#include "gdk/gdkx.h"
#include "gtk/gtk.h"
#define signals
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui_c.h"
#include <QWindow>

QWidget *cvEmbedWindow(const std::string &name);

#endif // QEMBEDDEDWINDOW_H
