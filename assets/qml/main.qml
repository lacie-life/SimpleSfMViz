import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.5

import AppEnums 1.0

Window {
    id: root
    visible: true
    width: QmlConst.DEFAULT_WIDTH
    height: QmlConst.DEFAULT_HEIGHT
    maximumHeight: QmlConst.DEFAULT_HEIGHT
    minimumHeight: QmlConst.DEFAULT_HEIGHT
    maximumWidth: QmlConst.DEFAULT_WIDTH
    minimumWidth: QmlConst.DEFAULT_WIDTH

    title: qsTr("GreenHouseAR")
    Loader {
        id: loader
        anchors.fill: parent
        source: QmlScreen.QML_MAIN_SCREEN
    }
}
