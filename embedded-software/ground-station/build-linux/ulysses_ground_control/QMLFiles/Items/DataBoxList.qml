import QtQuick

Rectangle {
    anchors.left: parent.left
    color: "transparent"
    implicitHeight: boxHeight
    height: implicitHeight

    property int size
    property int boxHeight: 50
    property var dataNames: []
    property var dataValues: []


    Repeater {
        model: size

        DataBox {
            required property int index
            dataName: dataNames[index]
            dataValue: dataValues[index]
            sections: size
            section_num: index+1
            height: boxHeight
        }
    }

}
