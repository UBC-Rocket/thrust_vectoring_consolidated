cmake_minimum_required(VERSION 3.21)

project(ulysses_ground_control
    VERSION 0.1.0
    LANGUAGES CXX C
)

include(GNUInstallDirs)

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
)

set(HDR_FILES
    "${HEAD_DIR}/SerialBridge.h"
    "${HEAD_DIR}/CommandSender.h"
    "${HEAD_DIR}/AlarmReceiver.h"
    "${HEAD_DIR}/SensorDataModel.h"
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

set_target_properties(ulysses_ground_control
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ----------------------------------------------------------------------
# QML module
# ----------------------------------------------------------------------
set_source_files_properties("${QML_DIR}/Items/Theme.qml"
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
        "${QML_DIR}/RadioOutputWindow.qml"
        "${QML_DIR}/RadioTestWindow.qml"
        "${QML_DIR}/Shortcuts.qml"
        "${QML_DIR}/Panels/Panel_State_And_Position.qml"
        "${QML_DIR}/Panels/Panel_Control.qml"
        "${QML_DIR}/Panels/Panel_Angles_And_Engine.qml"
        "${QML_DIR}/Panels/Panel_PID_Controller.qml"
        "${QML_DIR}/Panels/Panel_Rocket_Visualization.qml"
        "${QML_DIR}/Panels/Panel_System_Alert.qml"
        "${QML_DIR}/Panels/Panel_System_Health.qml"
        "${QML_DIR}/Items/BaseHeader.qml"
        "${QML_DIR}/Items/BasePanel.qml"
        "${QML_DIR}/Items/CommandCard.qml"
        "${QML_DIR}/Items/DataBox.qml"
        "${QML_DIR}/Items/DataBoxList.qml"
        "${QML_DIR}/Items/Theme.qml"
    RESOURCES
        "${MESH_FILES}"
        "${QML_DIR}/Items/qmldir"
)

# ----------------------------------------------------------------------
# Link Qt libraries
# ----------------------------------------------------------------------
target_link_libraries(ulysses_ground_control
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Qml
        Qt6::Quick
        Qt6::QuickControls2
        Qt6::Quick3D
        Qt6::SerialPort
        rocket-protocol
)

# ----------------------------------------------------------------------
# Install / package metadata
# ----------------------------------------------------------------------
install(TARGETS ulysses_ground_control
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE"
    DESTINATION ${CMAKE_INSTALL_DATADIR}/ulysses-ground-control
)

if(UNIX AND NOT APPLE)
    install(FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/distribution/Linux/ulysses-ground-control.desktop"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/applications
    )

    install(FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/distribution/Linux/ulysses-ground-control.svg"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps
    )
endif()

qt_import_qml_plugins(ulysses_ground_control)
qt_finalize_executable(ulysses_ground_control)

set(CPACK_PACKAGE_NAME "ulysses-ground-control")
set(CPACK_PACKAGE_VENDOR "UBC Rocket")
set(CPACK_PACKAGE_CONTACT "UBC Rocket")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Ground control station for UBC Rocket telemetry, command, and visualization"
)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}")

if(UNIX AND NOT APPLE)
    set(CPACK_GENERATOR "TGZ")
endif()

include(CPack)
