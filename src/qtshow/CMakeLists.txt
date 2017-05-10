if(WITH_QTSHOW)
  find_package(Qt5Widgets)
  find_package(Qt5OpenGL)
  add_definitions(-DWITH_FREEGLUT)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  add_executable(qtshow qtshow.cc MainWindow.cc GLWidget.cc)
  set_target_properties(qtshow PROPERTIES
    AUTOMOC ON
    AUTOUIC ON)
  qt5_use_modules(qtshow Widgets OpenGL)
  target_link_libraries(qtshow showstatic ${SHOW_LIBS} ${QT_LIBRARIES})
endif()
