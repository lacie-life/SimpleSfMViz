import QtQuick 2.0
import "Common"

QRec
{
    id: root

    // Properties
    property string sourceImage: ""
    property string label: ""
    property bool allowHover: false
    property bool allowText: false
    property bool allowImage: false
    property int sizeImage: witdth

    // Signals Declare
    signal clicked()
    signal pressed()
    signal released()
    signal pressAndHold()
    signal hoverIn()
    signal hoverOut()

    // Main
    QImage
    {
        id: imageID
        width: root.sizeImage
        anchors.centerIn: root
        imgName: root.sourceImage
        visible: (root.sourceImage !== CONST.EMPTY_STRING && root.allowImage )
    }

    QText
    {
        id: textID
        anchors.centerIn: root
        text: root.label
        visible: (root.label !== "" && root.allowText)
    }

    MouseArea
    {
        anchors.fill: root
        hoverEnabled: root.allowHover
        onClicked: root.clicked()
        onPressed: root.pressed()
        onPressAndHold: root.pressAndHold()
        onReleased: root.released()
        onEntered: root.hoverIn()
        onExited: root.hoverOut()
    }
}
