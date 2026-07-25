import QtQuick

Rectangle {
    property string dataName
    property double dataValue
    property int sections       //How many boxes are together
    property int section_num    //Which box is this one

    // EVA tile options: amber/green value variants and the 3px status left border.
    property color valueColor: Theme.textPrimary
    property color accentColor: "transparent"

    function formatDataValue(value) {
        const numericValue = Number(value)
        if (!Number.isFinite(numericValue)) {
            return "—"
        }
        return numericValue.toFixed(2)
    }

    height: 56
    width: (parent.width-18*sections)/sections
    x: (section_num-1) * (width+10)
    radius: 0
    border.color: Theme.divider
    border.width: Theme.strokeControl
    color: Theme.surfaceElevated

    Rectangle {
        visible: parent.accentColor.a > 0
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.margins: parent.border.width
        width: 3
        color: parent.accentColor
    }

    Text {
        id: name

        text: dataName.toUpperCase()
        font.family: Theme.fontFamily
        font.pixelSize: Theme.fontMetricLabel
        font.letterSpacing: 2
        color: Theme.textTertiary
        elide: Text.ElideRight
        width: parent.width - 20

        anchors.left: parent.left
        anchors.leftMargin: 10
        anchors.top: parent.top
        anchors.topMargin: 8
    }

    Text {
        id: value

        anchors.top: name.bottom
        anchors.topMargin: 2
        text: formatDataValue(dataValue)
        font.family: Theme.monoFamily
        font.pixelSize: Theme.fontMetricValue
        font.weight: Font.DemiBold
        color: parent.valueColor

        anchors.left: parent.left
        anchors.leftMargin: 10
    }

}
