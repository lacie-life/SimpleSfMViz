import QtQuick 2.0
import QtQuick.Controls 2.15

import "../../Component/Common"
import "../../Component"

import QmlCustomItem 1.0

QRec {
    id: root
    border.color: "red"

    // ComboBox {
    //     id: modelBox

    //     anchors.top: root.top
    //     anchors.topMargin: QmlConst.OFFSET*2

    //     anchors.left: root.left
    //     anchors.leftMargin: QmlConst.OFFSET*2

    //     anchors.right: root.right
    //     anchors.rightMargin: QmlConst.OFFSET*2

    //     textRole: "display"

    //     model: comboboxModel
    // }

    Button {
        id: cameraProcessButton

        width: 150
        height: 50

        text: qsTr("Camera Process")

        anchors.top: root.top
        anchors.topMargin: QmlConst.OFFSET*2

        anchors.left: root.left
        anchors.leftMargin: QmlConst.OFFSET

        anchors.right: root.right
        anchors.rightMargin: QmlConst.OFFSET

        onClicked: {
            // QmlHandler.qmlMessage(modelBox.currentIndex)
            // QmlModel.setDetectModel(parseInt(modelBox.currentIndex))

            QmlHandler.notifyQMLEvent(ENUMS.EVT_CLICK_CAMERA_RUN)
        }
    }

    Button {
        id: sfMProcessButton

        width: 150
        height: 50

        text: qsTr("SfM Process")

        anchors.top: cameraProcessButton.bottom
        anchors.topMargin: QmlConst.OFFSET*2

        anchors.left: root.left
        anchors.leftMargin: QmlConst.OFFSET

        anchors.right: root.right
        anchors.rightMargin: QmlConst.OFFSET

        onClicked: {
            // QmlHandler.qmlMessage(modelBox.currentIndex)
            // QmlModel.setDetectModel(parseInt(modelBox.currentIndex))

            QmlHandler.notifyQMLEvent(ENUMS.EVT_CLICK_SFM_RUN)
        }
    }

    Button {
        id: simepleSfMProcessButton

        width: 150
        height: 50

        text: qsTr("SimpleSfM Process")

        anchors.top: sfMProcessButton.bottom
        anchors.topMargin: QmlConst.OFFSET*2

        anchors.left: root.left
        anchors.leftMargin: QmlConst.OFFSET

        anchors.right: root.right
        anchors.rightMargin: QmlConst.OFFSET

        onClicked: {
            // QmlHandler.qmlMessage(modelBox.currentIndex)
            // QmlModel.setDetectModel(parseInt(modelBox.currentIndex))

            QmlHandler.notifyQMLEvent(ENUMS.EVT_CLICK_SIMPLESFM_RUN)
        }
    }
}
