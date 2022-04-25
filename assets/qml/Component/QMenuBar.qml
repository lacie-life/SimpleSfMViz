import QtQuick 2.0
import QmlCustomItem 1.0
import "Common"

QRec{
    id: root
    color: CONST.COLOR_MENU_BAR
    width: 80
    height: 680

    ListView
    {
        id: listScreen
        model: 5
        width: root.width
        height: root.height

        currentIndex: AppModel.currentScreenID
        interactive: contentHeight > height

        delegate: Item {
            id: delegateItem
            width: parent.width
            height: width

            QButton {
                id: icon

                allowImage: true
                anchors.fill: parent
                color: listScreen.currentIndex === index ? CONST.COLOR_MENU_BAR_FOCUS : CONST.COLOR_INVISIBLE

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
            return colorFolder + CONST.SEARCH_IMG
        case 1:
            return colorFolder + CONST.HOME_IMG
        case 2:
            return colorFolder + CONST.CONTROL_IMG
        case 3:
            return colorFolder + CONST.MAP_IMG
        case 4:
            return colorFolder + CONST.USER_IMG
        default:
            return ""
        }
    }

    function getEventID(index) {
        switch(index) {
        case 0:
            return ENUMS.UserClickSearch
        case 1:
            return ENUMS.UserClickHome
        case 2:
            return ENUMS.UserClickControl
        case 3:
            return ENUMS.UserClickMap
        case 4:
            return ENUMS.UserClickAccount
        default:
            return ""
        }
    }
}



