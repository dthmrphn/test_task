import QtQuick 2.15
import QtQuick.Window 2.15

import backend 1.0

Window {
    width: 800
    height: 480
    visible: true
    color: "#222222"
    title: qsTr("Weather Station")

    Backend {
        id: backend
    }

    Grid {
        id: grid
        width: 600
        height: 600
        spacing: 150

        SensorBar {
            id: sensorTemp
            progresNameValue: "Temperature"
            progressColor: "#f4201d"
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
