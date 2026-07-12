cmake_minimum_required(VERSION 3.21)

project(ulysses_ground_control
    VERSION 0.1.0
    LANGUAGES CXX C
)

# ----------------------------------------------------------------------
# For Linux release
include(GNUInstallDirs)
# ----------------------------------------------------------------------

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------------------------------------
# Shared protocol library (from libs/rocket-protocol/)
# ----------------------------------------------------------------------
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libs/rocket-protocol ${CMAKE_BINARY_DIR}/rocket-protocol)

# ----------------------------------------------------------------------
# Qt 6 modules
# ----------------------------------------------------------------------
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Gui
    Network
    Qml
    Quick
    QuickControls2
    Quick3D
    SerialPort
)

# ----------------------------------------------------------------------
# Paths (relative to project root)
# ----------------------------------------------------------------------
set(HEAD_DIR "HeadFiles")
set(SRC_DIR  "SourceFiles")
set(QML_DIR  "QMLFiles")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/${HEAD_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB_RECURSE MESH_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "QMLFiles/meshes/*.mesh")
# ----------------------------------------------------------------------
# C++ sources + headers
# ----------------------------------------------------------------------
set(SRC_FILES
    "${SRC_DIR}/main.cpp"
    "${SRC_DIR}/SerialBridge.cpp"
    "${SRC_DIR}/CommandSender.cpp"
    "${SRC_DIR}/AlarmReceiver.cpp"
    "${SRC_DIR}/SensorDataModel.cpp"
    "${SRC_DIR}/PresetManager.cpp"
)

set(HDR_FILES
    "${HEAD_DIR}/SerialBridge.h"
    "${HEAD_DIR}/CommandSender.h"
    "${HEAD_DIR}/AlarmReceiver.h"
    "${HEAD_DIR}/SensorDataModel.h"
    "${HEAD_DIR}/PresetManager.h"
)

# ----------------------------------------------------------------------
# Executable target
# ----------------------------------------------------------------------
qt_add_executable(ulysses_ground_control
    MACOSX_BUNDLE
    MANUAL_FINALIZATION
    ${SRC_FILES}
    ${HDR_FILES}
)

# ---------------------------------------------------------------------- 
# For Linux release
if(UNIX AND NOT APPLE)
    set_target_properties(ulysses_ground_control PROPERTIES 
    OUTPUT_NAME ulysses_gcs
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/../lib"
    )
endif()

install(TARGETS ulysses_ground_control
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

qt_generate_deploy_qml_app_script(
    TARGET ulysses_ground_control
    OUTPUT_SCRIPT deploy_script
)

install(SCRIPT ${deploy_script})

get_target_property(QT6_SERIALPORT_LOCATION Qt6::SerialPort LOCATION)
get_filename_component(QT6_LIB_DIR "${QT6_SERIALPORT_LOCATION}" DIRECTORY)

file(GLOB QT6_SERIALPORT_LIBS
    "${QT6_LIB_DIR}/libQt6SerialPort.so*"
)

install(FILES
    ${QT6_SERIALPORT_LIBS}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
# ----------------------------------------------------------------------


# Preset JSON lives in the repo so the whole team shares the same presets.
# PresetManager reads this macro at runtime.
target_compile_definitions(ulysses_ground_control PRIVATE
    ULYSSES_PRESETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/presets/pid_presets.json"
)

# ----------------------------------------------------------------------
# QML module
# ----------------------------------------------------------------------
set_source_files_properties("${QML_DIR}/Items/Theme.qml" "${QML_DIR}/Items/UiState.qml"
    PROPERTIES QT_QML_SINGLETON_TYPE TRUE
)

qt_add_qml_module(ulysses_ground_control
    URI ulysses_ground_control
    VERSION 1.0
    QML_FILES
        "${QML_DIR}/FullDroneAssembly.qml"
        "${QML_DIR}/Header.qml"
        "${QML_DIR}/LayoutGrid.qml"
        "${QML_DIR}/Main.qml"
        "${QML_DIR}/Shortcuts.qml"
        "${QML_DIR}/SideStripLeft.qml"
        "${QML_DIR}/SideStripRight.qml"
        "${QML_DIR}/BootOverlay.qml"
        "${QML_DIR}/EmergencyOverlay.qml"
        "${QML_DIR}/Panels/Panel_State_And_Position.qml"
        "${QML_DIR}/Panels/Panel_Control.qml"
        "${QML_DIR}/Panels/Panel_Angles_And_Engine.qml"
        "${QML_DIR}/Panels/Panel_PID_Controller.qml"
        "${QML_DIR}/Panels/Panel_Probe_Map.qml"
        "${QML_DIR}/Panels/Panel_Radio_Console.qml"
        "${QML_DIR}/Panels/Panel_Radio_Output.qml"
        "${QML_DIR}/Panels/Panel_Rocket_Visualization.qml"
        "${QML_DIR}/Panels/Panel_System_Alert.qml"
        "${QML_DIR}/Panels/Panel_System_Health.qml"
        "${QML_DIR}/Modes/Mode_Flight.qml"
        "${QML_DIR}/Modes/Mode_Tuning.qml"
        "${QML_DIR}/Modes/Mode_Map.qml"
        "${QML_DIR}/Modes/Mode_Diagnostics.qml"
        "${QML_DIR}/Items/BaseHeader.qml"
        "${QML_DIR}/Items/BasePanel.qml"
        "${QML_DIR}/Items/CommandCard.qml"
        "${QML_DIR}/Items/DataBox.qml"
        "${QML_DIR}/Items/DataBoxList.qml"
        "${QML_DIR}/Items/ModeBar.qml"
        "${QML_DIR}/Items/NumberField.qml"
        "${QML_DIR}/Items/PrimaryButton.qml"
        "${QML_DIR}/Items/Theme.qml"
        "${QML_DIR}/Items/ThemedCheckBox.qml"
        "${QML_DIR}/Items/ThemedScrollBar.qml"
        "${QML_DIR}/Items/ThemedTabButton.qml"
        "${QML_DIR}/Items/UiState.qml"
        "${QML_DIR}/Items/Blink.qml"
        "${QML_DIR}/Items/ClippedRect.qml"
        "${QML_DIR}/Items/ClippedButton.qml"
        "${QML_DIR}/Items/JpText.qml"
        "${QML_DIR}/Items/WarningStripes.qml"
        "${QML_DIR}/Items/Scanlines.qml"
        "${QML_DIR}/Items/GridBackdrop.qml"
        "${QML_DIR}/Items/SectionChip.qml"
    RESOURCES
        "${MESH_FILES}"
        "${QML_DIR}/Items/qmldir"
        "Resources/fonts/ChakraPetch-Regular.ttf"
        "Resources/fonts/ChakraPetch-Medium.ttf"
        "Resources/fonts/ChakraPetch-SemiBold.ttf"
        "Resources/fonts/ChakraPetch-Bold.ttf"
        "Resources/fonts/ShipporiMinchoB1-Bold.ttf"
        "Resources/fonts/ShipporiMinchoB1-ExtraBold.ttf"
        "Resources/fonts/JetBrainsMono-Regular.ttf"
        "Resources/fonts/JetBrainsMono-Bold.ttf"
        "Resources/images/asuka.png"
        "Resources/images/ubc-rocket-logo.png"
)

# ----------------------------------------------------------------------
# Link Qt libraries
# ----------------------------------------------------------------------
target_link_libraries(ulysses_ground_control
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Network
        Qt6::Qml
        Qt6::Quick
        Qt6::QuickControls2
        Qt6::Quick3D
        Qt6::SerialPort
        rocket-protocol
)

# Force the Basic (customizable) Controls style — see qtquickcontrols2.conf.
qt_add_resources(ulysses_ground_control "controls_conf"
    PREFIX "/"
    FILES qtquickcontrols2.conf
)

qt_import_qml_plugins(ulysses_ground_control)
qt_finalize_executable(ulysses_ground_control)

# ----------------------------------------------------------------------
# For Linux release
set(CPACK_PACKAGE_NAME "ulysses-gcs")
set(CPACK_PACKAGE_VERSION "0.1.1")  # Update this for each release
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Ulysses Ground Control")
set(CPACK_PACKAGE_VENDOR "UBC Rocket")
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
set(CPACK_VERBATIM_VARIABLES ON)
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ulysses-gcs")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Jimmy Pan")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6, libstdc++6, libgcc-s1, libxcb-cursor0, libxkbcommon-x11-0, qt6-wayland")

include(CPack)