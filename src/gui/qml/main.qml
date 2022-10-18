import QtQuick 2.15
import QtQuick.Window 2.15

import QRosNode 1.0

Window {
    width: 650
    height: 480
    visible: true
    color: "#222222"
    title: qsTr("Weather Station")

    QRosNode {
        id: backend
    }

    Grid {
        id: grid
        spacing: 40

        SensorBar {
            id: sensorTemp
            progresNameValue: "Temperature, °C"
            progressColor: "#f4201d"
            maxValue: 30
        }

        SensorBar {
            id: sensorHumy
            progresNameValue: "RelHumidity"
            decimal: 5
            maxValue: 1
        }
    }

    Connections {
        target: backend

        function onTemperatureReceived(value) {
            sensorTemp.value = value
        }

        function onRelhumidityReceived(value) {
            sensorHumy.value = value
        }
    }
}

/*##^##
Designer {
    D{i:0;height:600;width:800}D{i:2}D{i:3}D{i:1}
}
##^##*/
