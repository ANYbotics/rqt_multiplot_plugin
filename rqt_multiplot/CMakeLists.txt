cmake_minimum_required(VERSION 2.8.3)
project(rqt_multiplot)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rqt_gui
    rqt_gui_cpp
    variant_topic_tools
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

find_path(
  QWT_INCLUDE_DIRS
  NAMES
    qwt_plot.h
  PATHS
    /usr/include/qwt-qt4
    /usr/include/qwt
)
find_library(
  QWT_LIBRARIES
  NAMES
    qwt-qt4
    qwt
)

include(${QT_USE_FILE})

catkin_python_setup()

set(rqt_multiplot_SRCS
  src/multiplot_plugin.cpp
  src/plot_config_dialog.cpp
  src/plot_widget.cpp
)

set(rqt_multiplot_HDRS
  include/rqt_multiplot/multiplot_plugin.h
  include/rqt_multiplot/plot_config_dialog.h
  include/rqt_multiplot/plot_widget.h
)

set(rqt_multiplot_UIS
  src/multiplot_plugin.ui
  src/plot_config_dialog.ui
  src/plot_widget.ui
)

set(rqt_multiplot_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  LIBRARIES
    rqt_multiplot
  CATKIN_DEPENDS 
    roscpp
    rqt_gui
    rqt_gui_cpp
    variant_topic_tools
)

qt4_wrap_cpp(rqt_multiplot_MOCS ${rqt_multiplot_HDRS})
qt4_wrap_ui(rqt_multiplot_UIS_H ${rqt_multiplot_UIS})

include_directories(
  ${rqt_multiplot_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
  ${QWT_INCLUDE_DIRS}
)

add_library(rqt_multiplot
  ${rqt_multiplot_SRCS} 
  ${rqt_multiplot_MOCS}
  ${rqt_multiplot_UIS_H}
)

target_link_libraries(rqt_multiplot
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES} 
  ${QWT_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(rqt_multiplot)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rqt_multiplot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_multiplot
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_multiplot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
