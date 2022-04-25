import QtQuick 2.0
import QtQuick.Controls 2.15
import "Component"
import "Component/Common"

import QmlCustomItem 1.0

Item
{
    id: root

    QMenuBar {
        id: menuBar
        width: QmlConst.MENU_BAR_WIDTH
        height: root.width
        anchors.left: root.left
        anchors.top: root.top
    }

    QRec {
        id: mainScreen
        width: root.width - QmlConst.MENU_BAR_WIDTH
        height: root.height
        anchors.top: root.top
        anchors.left: menuBar.right

        color: QmlConst.COLOR_INVISIBLE
    }

    Loader {
        id: screenLoader
        visible: true
        property int screenID: QmlModel.currentScreenID
        anchors.fill: mainScreen
        onScreenIDChanged: source = getScreenUrl(screenID)
        Component.onCompleted: {
            source = getScreenUrl(screenID)
        }
    }

    function getScreenUrl(id){
        QmlHandler.qmlMessage("getScreenUrl " + id)
        switch(id){
        case ENUMS.HOME_SCREEN:
            QmlHandler.qmlMessage("Home Screen");
            return QmlScreen.QML_HOME
        case ENUMS.PROCESS_SCREEN:
            QmlHandler.qmlMessage("Process Screen");
            return QmlScreen.QML_PROCESS
        default:
            return ""
        }
    }
}
