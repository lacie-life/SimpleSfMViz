import QtQuick 2.0
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

import QmlCustomItem 1.0

import "../../Component/Common"
import "../../Component"

QRec {
    id: root
    border.color: "red"

    Image {
        id: imageViewer

        anchors.centerIn:  root.Center

        width: root.width
        height: root.height

        anchors.leftMargin: QmlConst.OFFSET
        anchors.rightMargin: QmlConst.OFFSET

        source: "qrc:/res/res/icon.svg"
        fillMode: Image.PreserveAspectFit

        property bool counter: false

        asynchronous: false
        cache: false

        function reload()
        {
//            console.log("Update counter")
            counter = !counter
            source = "image://live/image?id=" + counter
//            console.log(source)
        }
    }

    Connections{
        target: liveImageProvider

        function onImageChanged()
        {
            imageViewer.reload()
        }

    }
}
