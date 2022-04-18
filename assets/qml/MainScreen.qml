import QtQuick 2.0
import QtQuick.Controls 2.15

import QmlCustomItem 1.0

Item
{
    id: root

    Rectangle {
        id: mainScreen
        width: root.width - QmlConst.MENU_BAR_WIDTH
        height: root.height
        anchors.top: root.top

        color: QmlConst.COLOR_BACK_GROUND
    }

    Button{
        text: "SfM"
        anchors.centerIn: mainScreen.Center

        onClicked: {
            QmlHandler.qmlMessage("Run SfM");
            QmlHandler.qmlSendEvent(ENUMS.EVT_CLICK_RUN_SfM)
        }
    }
}
