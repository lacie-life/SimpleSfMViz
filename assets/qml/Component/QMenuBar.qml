import QtQuick 2.0
import QmlCustomItem 1.0
import "Common"

QRec{
    id: root
    color: QmlConst.COLOR_MENU_BAR
    width: 80
    height: 680

    ListView
    {
        id: listScreen
        model: 5
        width: root.width
        height: root.height

        currentIndex: QmlModel.currentScreenID
        interactive: contentHeight > height

        delegate: Item {
            id: delegateItem
            width: parent.width
            height: width

            QButton {
                id: icon

                allowImage: true
                anchors.fill: parent
                color: listScreen.currentIndex === index ? QmlConst.COLOR_MENU_BAR_FOCUS : QmlConst.COLOR_INVISIBLE

                sizeImage: width * 0.5
                sourceImage: getIcon(index, listScreen.currentIndex === index)

                onClicked: {
                    listScreen.currentIndex = index
                    QmlHandler.qmlSendEvent(getEventID(index))
                }
            }
        }

    }

    function getIcon(index, isDark) {
        var colorFolder = "light/"

        if (isDark) {
            colorFolder = "dark/"
        }

        switch(index) {
        case 0:
            return colorFolder + QmlConst.HOME_IMG
        case 1:
            return colorFolder + QmlConst.CONTROL_IMG
        default:
            return ""
        }
    }

    function getEventID(index) {
        switch(index) {
        case 0:
            return ENUMS.EVT_CLICK_HOME_SCREEN
        case 1:
            return ENUMS.EVT_CLICK_PROCESS_SCREEN
        default:
            return ""
        }
    }
}



