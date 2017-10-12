if (WITH_OPENGL)
  # Compile glui library
  SET(GLUI_DIR ${PROJECT_SOURCE_DIR}/3rdparty/glui-2.36/)
  add_library(glui_objects OBJECT
   ${GLUI_DIR}algebra3.cpp
   ${GLUI_DIR}arcball.cpp
   ${GLUI_DIR}glui_add_controls.cpp
   ${GLUI_DIR}glui_bitmap_img_data.cpp
   ${GLUI_DIR}glui_bitmaps.cpp
   ${GLUI_DIR}glui_button.cpp
   ${GLUI_DIR}glui_checkbox.cpp
   ${GLUI_DIR}glui_column.cpp
   ${GLUI_DIR}glui_commandline.cpp
   ${GLUI_DIR}glui_control.cpp
   ${GLUI_DIR}glui.cpp
   ${GLUI_DIR}glui_edittext.cpp
   ${GLUI_DIR}glui_filebrowser.cpp
   ${GLUI_DIR}glui_listbox.cpp
   ${GLUI_DIR}glui_list.cpp
   ${GLUI_DIR}glui_mouse_iaction.cpp
   ${GLUI_DIR}glui_node.cpp
   ${GLUI_DIR}glui_panel.cpp
   ${GLUI_DIR}glui_radio.cpp
   ${GLUI_DIR}glui_rollout.cpp
   ${GLUI_DIR}glui_rotation.cpp
   ${GLUI_DIR}glui_scrollbar.cpp
   ${GLUI_DIR}glui_separator.cpp
   ${GLUI_DIR}glui_spinner.cpp
   ${GLUI_DIR}glui_statictext.cpp
   ${GLUI_DIR}glui_string.cpp
   ${GLUI_DIR}glui_textbox.cpp
   ${GLUI_DIR}glui_translation.cpp
   ${GLUI_DIR}glui_tree.cpp
   ${GLUI_DIR}glui_treepanel.cpp
   ${GLUI_DIR}glui_window.cpp
   ${GLUI_DIR}quaternion.cpp
   ${GLUI_DIR}viewmodel.cpp
  )
  target_include_directories(glui_objects PUBLIC ${PROJECT_SOURCE_DIR}/3rdparty/glui-2.36/include/)

  # compile with -fPIC so that the result can be made into a shared library
  set_property(TARGET glui_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

  add_library(glui_static STATIC $<TARGET_OBJECTS:glui_objects>)
  add_library(glui_shared SHARED $<TARGET_OBJECTS:glui_objects>)
  if (WIN32)
    target_link_libraries(glui_static freeglut)
  endif(WIN32)
  set_target_properties(glui_static PROPERTIES OUTPUT_NAME glui CLEAN_DIRECT_OUTPUT 1)
  set_target_properties(glui_shared PROPERTIES OUTPUT_NAME glui CLEAN_DIRECT_OUTPUT 1)
endif()

# Compile wxthings library
if (WITH_WXWIDGETS)
  find_package(wxWidgets REQUIRED core base gl)
  # wxWidgets include (this will do all the magic to configure everything)
  include( ${wxWidgets_USE_FILE})
  SET(WXTHINGS_DIR ${PROJECT_SOURCE_DIR}/3rdparty/wxthings/src/)
  add_library(wxthings STATIC 
    ${WXTHINGS_DIR}block.cpp
    ${WXTHINGS_DIR}bmpcombo.cpp
    ${WXTHINGS_DIR}dropdown.cpp
    # ${WXTHINGS_DIR}filebrws.cpp  # these two are not needed and would create unnecessary dependencies
    #${WXTHINGS_DIR}genergdi.cpp
    ${WXTHINGS_DIR}geometry.cpp
    ${WXTHINGS_DIR}menubtn.cpp
    ${WXTHINGS_DIR}optvalue.cpp
    ${WXTHINGS_DIR}range.cpp
    ${WXTHINGS_DIR}spinctld.cpp
    ${WXTHINGS_DIR}toggle.cpp
  )
  target_include_directories(wxthings PUBLIC ${PROJECT_SOURCE_DIR}/3rdparty/wxthings/include)
endif()

if (WITH_LASLIB)
# Compile las library
set(CMAKE_CXX_FLAGS "-DNDEBUG -DUNORDERED -std=c++0x -fPIC" )
SET(LASLIB_DIR ${PROJECT_SOURCE_DIR}/3rdparty/lastools/laslib/src/)
FILE(GLOB LASLIB_SRCS "${LASLIB_DIR}/*.cpp" "${LASLIB_DIR}/*.hpp")
add_library(laslib_objects OBJECT ${LASLIB_SRCS})
target_include_directories(laslib_objects PUBLIC ${PROJECT_SOURCE_DIR}/3rdparty/lastools/laslib/inc)

# compile with -fPIC so that the result can be made into a shared library
set_property(TARGET laslib_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(laslib_static STATIC $<TARGET_OBJECTS:laslib_objects>)
add_library(laslib_shared SHARED $<TARGET_OBJECTS:laslib_objects>)

set_target_properties(laslib_static PROPERTIES OUTPUT_NAME laslib CLEAN_DIRECT_OUTPUT 1)
set_target_properties(laslib_shared PROPERTIES OUTPUT_NAME laslib CLEAN_DIRECT_OUTPUT 1)

endif(WITH_LASLIB)

if (WITH_GLEE)
# Compile GLEE library 
IF (APPLE)
  FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation )
ENDIF (APPLE)

SET(GLEE_DIR ${PROJECT_SOURCE_DIR}/3rdparty/glee/)

add_library(glee_static STATIC ${GLEE_DIR}GLee.c)
add_library(glee_shared SHARED ${GLEE_DIR}GLee.c)

IF(APPLE)
  target_link_libraries(glee_static ${COREFOUNDATION_LIBRARY} ${OPENGL_LIBRARIES})
  target_link_libraries(glee_shared ${COREFOUNDATION_LIBRARY} ${OPENGL_LIBRARIES})
ENDIF(APPLE)

set_target_properties(glee_static PROPERTIES OUTPUT_NAME glee CLEAN_DIRECT_OUTPUT 1)
set_target_properties(glee_shared PROPERTIES OUTPUT_NAME glee CLEAN_DIRECT_OUTPUT 1)
endif()

IF(WIN32)
  add_subdirectory(libzip-1.0.1)
ENDIF()

# Compile AprilTags library if required
add_subdirectory(apriltag-2016-12-01)

add_subdirectory(rply-1.1.4)
