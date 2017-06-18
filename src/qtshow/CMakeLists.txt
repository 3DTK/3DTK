if(WITH_QTSHOW)
  find_package(Qt5Widgets)
  find_package(Qt5OpenGL)
  add_definitions(-DWITH_FREEGLUT)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  qt5_wrap_cpp(QTSHOW_HEADERS_MOC
    ${CMAKE_SOURCE_DIR}/include/qtshow/GLWidget.h
    ${CMAKE_SOURCE_DIR}/include/qtshow/MainWindow.h
    ${CMAKE_SOURCE_DIR}/include/qtshow/QtShow.h)

  qt5_wrap_ui(QTSHOW_FORMS_HEADERS MainWindow.ui)

  add_executable(qtshow main.cc QtShow.cc MainWindow.cc GLWidget.cc ${QTSHOW_HEADERS_MOC} ${QTSHOW_FORMS_HEADERS})

  qt5_use_modules(qtshow Widgets OpenGL)
  target_link_libraries(qtshow showstatic ${SHOW_LIBS} ${QT_LIBRARIES})
endif()
