#ifndef QPROGRESSBARDIALOG_H
#define QPROGRESSBARDIALOG_H

#include <QObject>
#include <QFutureWatcher>

class QProgressBarDialog : public QObject
{
    Q_OBJECT
    Q_PROPERTY(float progress READ progress NOTIFY progressChanged)
    Q_PROPERTY(bool running READ running NOTIFY runningChanged)

public:
    explicit QProgressBarDialog(QObject *parent = nullptr);
    float progress();
    bool running();

public slots:
    void startComputation();
    void cancelComputation();

private slots:
    void updateProgress(int value);

signals:
    void progressChanged();
    void runningChanged();

private:
    bool m_running = false;
    int m_progressValue = 0;
    QVector<int> vector;
    QObject m_Model;
    QFutureWatcher<void> m_futureWatcher;

};

#endif // QPROGRESSBARDIALOG_H
