import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0

Rectangle {
    id: cameraControls
    property var camera

    border.color: "#000000"
    border.width: 2
    radius: 5
    color: "#55ffffff"

    width: parent ? parent.width - 10 : 400
    height: 150

    Component.onCompleted: if (camera) actualStuff.createObject(cameraControls)

    Component {
        id: actualStuff
        GridLayout {
            anchors.fill: parent
            anchors.margins: 5
            columns: 3

            Label { text: "Azimuth" }
            Slider {
                Layout.fillWidth: true
                from: 0
                to: 360
                value: 180
                onValueChanged: cameraControls.camera.azimuth = value
            }
            Label { text: cameraControls.camera.azimuth.toFixed(2) }

            Label { text: "Elevation" }
            Slider {
                Layout.fillWidth: true
                from: 0
                to: 90
                value: 10
                onValueChanged: cameraControls.camera.elevation = value
            }
            Label { text: cameraControls.camera.elevation.toFixed(2) }

            Label { text: "Distance" }
            Slider {
                id: distanceSlider
                Layout.fillWidth: true
                from: 1
                to: 25
                value: 15
                onValueChanged: cameraControls.camera.distance = value
            }
            Label { text: cameraControls.camera.distance.toFixed(2) }
        }
    }
}
