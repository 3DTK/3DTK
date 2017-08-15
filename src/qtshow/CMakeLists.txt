if(WITH_QTSHOW)
  find_package(Qt5Widgets)
  find_package(Qt5OpenGL)
  add_definitions(-DWITH_FREEGLUT)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  qt5_wrap_cpp(QTSHOW_HEADERS_MOC
    ${PROJECT_SOURCE_DIR}/include/qtshow/GLWidget.h
    ${PROJECT_SOURCE_DIR}/include/qtshow/MainWindow.h
    ${PROJECT_SOURCE_DIR}/include/qtshow/QtShow.h
    ${PROJECT_SOURCE_DIR}/include/qtshow/ScanPicker.h)

  qt5_wrap_cpp(GLWIDGET_HEADERS_MOC
    ${PROJECT_SOURCE_DIR}/include/qtshow/GLWidget.h)

  qt5_wrap_ui(QTSHOW_FORMS_HEADERS
    MainWindow.ui
    ScanPicker.ui)

  add_executable(qtshow
    main.cc
    GLWidget.cc
    MainWindow.cc
    QtShow.cc
    ScanPicker.cc
    ${QTSHOW_HEADERS_MOC}
    ${QTSHOW_FORMS_HEADERS})

  add_library(glwidget STATIC GLWidget.cc ${GLWIDGET_HEADERS_MOC})
    
  qt5_use_modules(qtshow Widgets OpenGL)
  target_link_libraries(qtshow showstatic ${SHOW_LIBS} ${QT_LIBRARIES})
  qt5_use_modules(glwidget Widgets OpenGL)
  target_link_libraries(glwidget showstatic ${SHOW_LIBS} ${QT_LIBRARIES})
endif()
