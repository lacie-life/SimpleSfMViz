#ifndef CONSTANT_H
#define CONSTANT_H

#include <QObject>
#include <QString>
#include <QUrl>
#include <QColor>
#include <QMutex>
#include <QCoreApplication>

#ifndef MACRO_DEFINE
#define MACRO_DEFINE

#define CONSOLE qDebug() << "[" << __FUNCTION__ << "] "
#define DEF_VAR(type, name, value) Q_PROPERTY(type name READ name NOTIFY dataUpdate) \
    public: type name() { return value; }

#define DEF_CONST(type, name, value) Q_PROPERTY(type name READ name CONSTANT) \
    public: type name() const { return value; }

#endif

#define DEFS AppConstant::instance()
#define DELETE_DEFS AppConstant::DestroyInstance()

#ifndef BUILD_DIR
#define BUILD_DIR QCoreApplication::applicationDirPath()
#endif

class AppConstant : public QObject
{
    Q_OBJECT

    /************************* DEFINE QML URL *****************************/
    DEF_CONST(QUrl     , QML_MAIN_URL               , QUrl(QStringLiteral("qrc:/qml/main.qml")))

    /************************* DEFINE SOMETHINGS *****************************/
    DEF_CONST(int      , DEFAULT_WIDTH               , 1280)
    DEF_CONST(int      , DEFAULT_HEIGHT              , 720)
    DEF_CONST(int      , DEFAULT_BTN_W               , 426)
    DEF_CONST(int      , DEFAULT_BTN_H               , 150)

signals:
    void dataUpdated();

public:
    static AppConstant* instance()
    {
        m_lock.lock();
        if(nullptr == m_instance){
            m_instance = new AppConstant();
        }
        m_lock.unlock();
        return m_instance;
    }

    static void DestroyInstance()
    {
        m_lock.lock();
        if (nullptr != m_instance){
            delete m_instance;
        }
        m_instance = nullptr;
        m_lock.unlock();
    }

private:
    explicit AppConstant(QObject* parent = nullptr) : QObject { parent } {}
    ~AppConstant() {}
    AppConstant(const AppConstant&) = delete;
    bool operator= (const AppConstant&) = delete;

    static AppConstant* m_instance;
    static QMutex m_lock;

    /********************************************** Point Cloud Resource ********************************/

    DEF_CONST(QString, POINT_CLOUD_EXAMPLE   , BUILD_DIR + "/assest/bunny.ply")
    DEF_CONST(QString, POINT_CLOUD_VERTEX    , BUILD_DIR + "/assest/shader/pointcloud.vert")
    DEF_CONST(QString, POINT_CLOUD_FRAGMENT  , BUILD_DIR + "/assest/shader/pointcloud.frag")

    /********************************************** GENERAL **********************************************/
    DEF_CONST(int, MAX_WIDTH        , 1280  )
    DEF_CONST(int, MAX_HEIGHT       , 680   )
    DEF_CONST(int, MENU_BAR_WIDTH   , 80    )

    DEF_CONST(QString   , EMPTY_STRING , ""    )

    // search screen constant
    DEF_CONST(int, INPUT_BOX_WIDTH      , 1100  )
    DEF_CONST(int, INPUT_BOX_HEIGHT     , 60    )
    DEF_CONST(int, INPUT_BOX_TOP_MARGIN , 15    )

    // constant color
    DEF_CONST(QColor, COLOR_SWITCH_OFF      , "#ADADAD")
    DEF_CONST(QColor, COLOR_SWITCH_ON       , "#00CCD9")
    DEF_CONST(QColor, COLOR_SWITCH_BTN_NOOD , "#F0F0F0")
    DEF_CONST(QColor, COLOR_BORDER_DARK     , "#777777")

    DEF_CONST(QColor, COLOR_MENU_BAR        , "#00AF2A")
    DEF_CONST(QColor, COLOR_MENU_BAR_FOCUS  , "#DDDDDD")
    DEF_CONST(QColor, COLOR_BACK_GROUND     , COLOR_MENU_BAR_FOCUS())

    DEF_CONST(QString, COLOR_INVISIBLE      , "transparent")

    // QChart define
    DEF_CONST(int, CHART_DRAW_OFFSET, 10)
};

#endif // CONSTANT_H
