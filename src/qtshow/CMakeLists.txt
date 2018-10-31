if (NOT MSVC AND WITH_OPENGL AND WITH_QT)
  find_package(Qt5Widgets)
  find_package(Qt5OpenGL)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/glui-2.36/include/)
  add_definitions(-DWITH_OPENGL)

  qt5_wrap_cpp(QTSHOW_HEADERS_MOC
    ${PROJECT_SOURCE_DIR}/include/qtshow/GLWidget.h
    ${PROJECT_SOURCE_DIR}/include/qtshow/MainWindow.h
    ${PROJECT_SOURCE_DIR}/include/qtshow/NavPushButton.h
    ${PROJECT_SOURCE_DIR}/include/qtshow/QtShow.h
    ${PROJECT_SOURCE_DIR}/include/qtshow/ScanPicker.h
    ${PROJECT_SOURCE_DIR}/include/qtshow/SettingsDialog.h
    ${PROJECT_SOURCE_DIR}/include/qtshow/ShortcutsDialog.h)

  qt5_wrap_cpp(GLWIDGET_HEADERS_MOC
    ${PROJECT_SOURCE_DIR}/include/qtshow/GLWidget.h)

  qt5_wrap_ui(QTSHOW_FORMS_HEADERS
    MainWindow.ui
    ScanPicker.ui
    SettingsDialog.ui
    ShortcutsDialog.ui)

  add_executable(qtshow
    main.cc
    MainWindow.cc
    NavPushButton.cc
    QtShow.cc
    ScanPicker.cc
    SettingsDialog.cc
    ShortcutsDialog.cc
    ${QTSHOW_HEADERS_MOC}
    ${QTSHOW_FORMS_HEADERS})

  add_library(glwidget STATIC GLWidget.cc ${GLWIDGET_HEADERS_MOC})

  target_link_libraries(glwidget Qt5::Widgets Qt5::OpenGL)
  target_link_libraries(glwidget show_shared ${SHOW_LIBS_SHARED} ${QT_LIBRARIES})

  target_link_libraries(qtshow Qt5::Widgets)
  target_link_libraries(qtshow glwidget ${QT_LIBRARIES})
endif()
