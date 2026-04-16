import QtQuick

Rectangle {
    anchors.left: parent.left
    color: "transparent"
    implicitHeight: boxHeight
    height: implicitHeight

    property int size
    property int boxHeight: 50
    property list<string> dataNames
    // `var` (not `double`) so callers can pass NaN / null to indicate "no data"
    // without hitting list<double> coercion. DataBox.formatDataValue renders "—".
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
