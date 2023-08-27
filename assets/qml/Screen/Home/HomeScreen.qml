import QtQuick 2.0
import QtQuick.Controls 2.15
import QtQuick.Dialogs 1.3

import QmlCustomItem 1.0

import "../../Component/Common"
import "../../Component"

QRec {
    id: homeScreen

    Button {
        id: configPath

        text: qsTr("Choose path")

        width: 200
        height: 50

        anchors.top: homeScreen.top
        anchors.topMargin: QmlConst.OFFSET

        anchors.right: homeScreen.right
        anchors.rightMargin: QmlConst.OFFSET + configBox.width - configPath.width

        anchors.bottom: configBox.top
        anchors.bottomMargin: QmlConst.OFFSET
    }

    ImageView {
        id: imageView
        anchors.left: homeScreen.left
        anchors.leftMargin: QmlConst.OFFSET

        anchors.bottom: homeScreen.bottom
        anchors.bottomMargin: QmlConst.OFFSET

        anchors.top: homeScreen.top
        anchors.topMargin: QmlConst.OFFSET * 5

        anchors.right: configBox.left
        anchors.rightMargin: QmlConst.OFFSET
    }

    ConfigBox {
        id: configBox

        width: 200

        enabled: false

        anchors.right: homeScreen.right
        anchors.rightMargin: QmlConst.OFFSET

        anchors.top: homeScreen.top
        anchors.topMargin: QmlConst.OFFSET * 5

        anchors.bottom: homeScreen.bottom
        anchors.bottomMargin: QmlConst.OFFSET
    }


    FileDialog {
        id: chooseConfig

        title: "Please choose a file"

        folder: shortcuts.home

        width: 600
        height: 400

        onAccepted: {
            QmlHandler.qmlMessage("Path: " + chooseConfig.fileUrl)
            QmlModel.setSfMConfigPath(chooseConfig.fileUrl)
            QmlHandler.notifyQMLEvent(ENUMS.EVT_CLICK_CHOOSE_CONFIG)
            chooseConfig.close()
            configBox.enabled = true
        }
    }

    Connections {
        target: configPath
        function onClicked(mouse) {
            chooseConfig.open()
        }
    }
}
