import QtQuick 2.0
import QtQuick.Controls 2.15

import "../../Component/Common"
import "../../Component"

import QmlCustomItem 1.0

QRec {
    id: root
    border.color: "red"

    ComboBox {
        id: modelBox

        anchors.top: root.top
        anchors.topMargin: QmlConst.OFFSET*2

        anchors.left: root.left
        anchors.leftMargin: QmlConst.OFFSET*2

        anchors.right: root.right
        anchors.rightMargin: QmlConst.OFFSET*2

        textRole: "display"

        model: comboboxModel
    }

    Button {
        id: processButton

        width: 100
        height: 50

        text: qsTr("Process")

        anchors.bottom: root.bottom
        anchors.bottomMargin: QmlConst.OFFSET*2

        anchors.left: root.left
        anchors.leftMargin: QmlConst.OFFSET*2

        anchors.right: root.right
        anchors.rightMargin: QmlConst.OFFSET*2

        onClicked: {
            QmlHandler.qmlMessage(modelBox.currentIndex)

            QmlModel.setDetectModel(parseInt(modelBox.currentIndex))

            QmlHandler.notifyQMLEvent(ENUMS.EVT_CLICK_RUN)
        }
    }
}
