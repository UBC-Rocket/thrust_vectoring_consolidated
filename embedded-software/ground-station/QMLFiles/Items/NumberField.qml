import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Basic as Basic
import QtQuick.Layouts

// Labeled decimal input whose `text` is only overwritten by external `value`
// updates when the numeric value ACTUALLY differs from what the user typed.
// This preserves trailing zeros / user formatting (e.g. "1.200" stays "1.200"
// on blur even though Number("1.200") === 1.2), while still refreshing when
// a preset load or similar external write changes the underlying number.
ColumnLayout {
    id: root

    property string label: ""
    property real   value: 0.0
    property int    decimals: 6
    property bool   fieldEnabled: true

    // Emitted only on user edits (not when `value` changes externally).
    signal valueEdited(real newValue)

    spacing: 4
    Layout.fillWidth: true
    Layout.minimumWidth: 0
    Layout.preferredWidth: 0

    function _sanitized(raw, fallback) {
        const parsed = Number(raw)
        return isNaN(parsed) ? fallback : parsed
    }

    Text {
        text: root.label
        visible: root.label.length > 0
        color: Theme.textSecondary
        font.family: Theme.fontFamily
        font.pixelSize: Theme.fontBody
        font.bold: true
    }

    Basic.TextField {
        id: input
        Layout.fillWidth: true
        Layout.minimumWidth: 0
        Layout.preferredWidth: 0
        Layout.preferredHeight: 36
        leftPadding: 10
        rightPadding: 10
        enabled: root.fieldEnabled
        color: Theme.textPrimary
        font.family: Theme.monoFamily
        font.pixelSize: Theme.fontBody
        inputMethodHints: Qt.ImhFormattedNumbersOnly
        validator: DoubleValidator { decimals: root.decimals }
        selectByMouse: true

        background: Rectangle {
            radius: Theme.radiusCard
            color: Theme.background
            border.width: Theme.strokeControl
            border.color: input.activeFocus ? Theme.accent : Theme.border
        }

        // Seed once on creation so the field shows the initial value.
        Component.onCompleted: text = String(root.value)

        // User edits only (programmatic text changes don't fire onTextEdited).
        onTextEdited: root.valueEdited(root._sanitized(text, root.value))
    }

    // Refresh the displayed text from `value` only when:
    //  - the field is not being edited, AND
    //  - the numeric value actually differs from what's in the field.
    // Without the numeric-equality check, blurring "1.200" would snap it to "1.2"
    // because String(1.2) === "1.2" — the user's trailing zeros disappear.
    Binding {
        target: input
        property: "text"
        value: String(root.value)
        when: !input.activeFocus && Number(input.text) !== root.value
    }
}
