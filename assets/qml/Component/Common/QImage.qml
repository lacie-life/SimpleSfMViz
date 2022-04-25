import QtQuick 2.0

Image {
    id: root
    property alias backgroundColor: bg.color

    QRec
    {
        id: bg
        anchors.fill: parent
        color: CONST.COLOR_INVISIBLE
    }

    property string imgName: CONST.EMPTY_STRING

    source: imgName !== CONST.EMPTY_STRING ? CONST.IMAGE_FOLDER + root.imgName
                                           : CONST.EMPTY_STRING

}
