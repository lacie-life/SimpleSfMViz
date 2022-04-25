import QtQuick 2.0

Text
{
    id: root
    property bool isBold: false
    property int fontSize: 25

    font.bold: isBold
    font.pixelSize: fontSize
    text: ""
    color: "#ffffff"
}
