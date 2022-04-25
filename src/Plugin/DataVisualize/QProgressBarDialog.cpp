#include "DataVisualize/QProgressBarDialog.h"
#include <QtConcurrent/QtConcurrentMap>

QProgressBarDialog::QProgressBarDialog(QObject *parent)
    : QObject{parent}
{

}

void spin(int &iteration)
{
    const int work = 1000 * 1000 * 40;
    for(int j = 0; j < work; ++j)
        iteration *= 2;
}

float QProgressBarDialog::progress()
{
    return m_progressValue;
}

bool QProgressBarDialog::running()
{
    return m_running;
}

void QProgressBarDialog::startComputation()
{
    m_running = true;
    emit runningChanged();
    // Prepare the vector
    vector.clear();
    for(int i = 0; i < 40; ++i)
        vector.append(i);
    const QFuture<void> future = QtConcurrent::map(vector, spin);
    m_futureWatcher.setFuture(future);
    connect(&m_futureWatcher, &QFutureWatcher<void>::progressValueChanged,
            this, &QProgressBarDialog::updateProgress);
}

void QProgressBarDialog::cancelComputation()
{
    m_running = false;
    emit runningChanged();
}

void QProgressBarDialog::updateProgress(int value)
{
    m_progressValue = value;
    emit progressChanged();
}
