import QtQuick 2.15
import QtQuick.Shapes 1.15
import QtGraphicalEffects 1.15

Item {
    id: progress
    implicitWidth: 300
    implicitHeight: 300

    property string progresNameValue: ""

    property real maxValue: 100
    property real value: 1

    property int samples: 12

    property bool roundCap: true
    property int strokeWidth: 15
    property int startAngle: -210
    property int sweepAngle: 240

    property color bgColor: "#00000000"
    property color bgStrokeColor: "#b0cfde"
    property color progressColor: "#55ffff"
    property int progressWidth: 15

    property string textValue: ""
    property bool textShowValue: true
    property string textFont: "Segoe UI"
    property color textColor: "#3A8BC3"
    property int textSize: 45

    property int lastValue: 0

    property int decimal: 1

    Shape {
        id: shape
        anchors.fill: parent
        layer.enabled:  true
        layer.samples: progress.samples
        antialiasing: true

        ShapePath {
            id: pathBG
            strokeColor: progress.bgStrokeColor
            fillColor: progress.bgColor
            strokeWidth:  progress.strokeWidth / 4
            capStyle: progress.roundCap ? ShapePath.RoundCap : ShapePath.FlatCap

            PathAngleArc {
                radiusX: (progress.width / 2) - (progress.progressWidth / 2)
                radiusY: (progress.height / 2) - (progress.progressWidth / 2)
                centerX: progress.width / 2
                centerY: progress.height / 2
                startAngle: progress.startAngle
                sweepAngle: progress.sweepAngle
            }
        }

        ShapePath {
            id: path
            strokeColor: progress.progressColor
            fillColor: progress.bgColor
            strokeWidth:  progress.strokeWidth
            capStyle: progress.roundCap ? ShapePath.RoundCap : ShapePath.FlatCap

            PathAngleArc {
                radiusX: (progress.width / 2) - (progress.progressWidth / 2)
                radiusY: (progress.height / 2) - (progress.progressWidth / 2)
                centerX: progress.width / 2
                centerY: progress.height / 2
                startAngle: progress.startAngle
                sweepAngle: (progress.sweepAngle / progress.maxValue) * progress.value

                NumberAnimation on sweepAngle {to: progress.value} 
                Behavior on sweepAngle { NumberAnimation {} }
            }
        }

        Text {
            id: textProgressValue
            text: progress.value.toFixed(progress.decimal)
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            color: progress.textColor
            font.family: progress.textFont
            font.pointSize: progress.textSize
        }

        Text {
            id: textProgressName
            text: progress.progresNameValue
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenterOffset: 60
            color: progress.textColor
            font.family: progress.textFont
            font.pointSize: 20
        }
    }
}

/*##^##
Designer {
    D{i:0;height:250;width:250}D{i:2}D{i:4}D{i:6}D{i:1}
}
##^##*/
