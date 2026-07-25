import QtQuick

// Stepped blink (no easing): opacity 1 ↔ lowOpacity on a fixed period.
// Attach to any Item:  Blink { target: recDot; period: 1200 }
SequentialAnimation {
    id: anim

    property Item target
    property int period: Theme.blinkPeriod
    property real lowOpacity: 0.15

    running: target !== null && target.visible
    loops: Animation.Infinite

    PropertyAction { target: anim.target; property: "opacity"; value: 1 }
    PauseAnimation { duration: anim.period / 2 }
    PropertyAction { target: anim.target; property: "opacity"; value: anim.lowOpacity }
    PauseAnimation { duration: anim.period / 2 }
}
