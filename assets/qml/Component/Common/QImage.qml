import QtQuick 2.0

Image {
    id: root
    property alias backgroundColor: bg.color

    QRec
    {
        id: bg
        anchors.fill: parent
        color: QmlConst.COLOR_INVISIBLE
    }

    property string imgName: QmlConst.EMPTY_STRING

    source: imgName !== QmlConst.EMPTY_STRING ? QmlConst.IMAGE_FOLDER + root.imgName
                                           : QmlConst.EMPTY_STRING

}
