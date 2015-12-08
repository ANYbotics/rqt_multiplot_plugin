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
  src/BitOperations.cpp
  src/BoundingRectangle.cpp
  src/ColorOperations.cpp
  src/ConfigComboBox.cpp
  src/CurveAxisConfig.cpp
  src/CurveAxisConfigWidget.cpp
  src/CurveAxisScale.cpp
  src/CurveAxisScaleWidget.cpp
  src/CurveColor.cpp
  src/CurveColorWidget.cpp
  src/CurveConfig.cpp
  src/CurveConfigDialog.cpp
  src/CurveConfigWidget.cpp
  src/CurveData.cpp
  src/CurveDataCircularBuffer.cpp
  src/CurveDataConfig.cpp
  src/CurveDataConfigWidget.cpp
  src/CurveDataList.cpp
  src/CurveDataSequencer.cpp
  src/CurveDataVector.cpp
  src/CurveItemWidget.cpp
  src/CurveListWidget.cpp
  src/CurveStyle.cpp
  src/CurveStyleWidget.cpp
  src/FileScheme.cpp
  src/Message.cpp
  src/MessageDefinitionLoader.cpp
  src/MessageFieldCompleter.cpp
  src/MessageFieldItem.cpp
  src/MessageFieldItemModel.cpp
  src/MessageFieldLineEdit.cpp
  src/MessageFieldSubscriber.cpp
  src/MessageFieldSubscriberRegistry.cpp
  src/MessageFieldTreeWidget.cpp
  src/MessageFieldWidget.cpp
  src/MessageSubscriber.cpp
  src/MessageSubscriberRegistry.cpp
  src/MessageTopicComboBox.cpp
  src/MessageTopicRegistry.cpp
  src/MessageTypeComboBox.cpp
  src/MessageTypeRegistry.cpp
  src/MultiplotConfig.cpp
  src/MultiplotConfigWidget.cpp
  src/MultiplotPlugin.cpp
  src/MultiplotWidget.cpp
  src/PackageRegistry.cpp
  src/PackageScheme.cpp
  src/PenStyleComboBox.cpp
  src/PenStyleItemDelegate.cpp
  src/PlotAxesConfig.cpp
  src/PlotAxesConfigWidget.cpp
  src/PlotAxisConfig.cpp
  src/PlotAxisConfigWidget.cpp
  src/PlotConfig.cpp
  src/PlotConfigDialog.cpp
  src/PlotConfigWidget.cpp
  src/PlotCursor.cpp
  src/PlotCursorMachine.cpp
  src/PlotCurve.cpp
  src/PlotLegend.cpp
  src/PlotLegendConfig.cpp
  src/PlotLegendConfigWidget.cpp
  src/PlotMagnifier.cpp
  src/PlotPanner.cpp
  src/PlotTableConfig.cpp
  src/PlotTableConfigWidget.cpp
  src/PlotTableWidget.cpp
  src/PlotWidget.cpp
  src/PlotZoomer.cpp
  src/PlotZoomerMachine.cpp
  src/StatusWidget.cpp
  src/UrlComboBox.cpp
  src/UrlCompleter.cpp
  src/UrlItem.cpp
  src/UrlItemModel.cpp
  src/UrlScheme.cpp
  src/XmlSettings.cpp
)

set(rqt_multiplot_HDRS
  include/rqt_multiplot/BitOperations.h
  include/rqt_multiplot/BoundingRectangle.h
  include/rqt_multiplot/ColorOperations.h
  include/rqt_multiplot/ConfigComboBox.h
  include/rqt_multiplot/CurveAxisConfig.h
  include/rqt_multiplot/CurveAxisConfigWidget.h
  include/rqt_multiplot/CurveAxisScale.h
  include/rqt_multiplot/CurveAxisScaleWidget.h
  include/rqt_multiplot/CurveColor.h
  include/rqt_multiplot/CurveColorWidget.h
  include/rqt_multiplot/CurveConfig.h
  include/rqt_multiplot/CurveConfigDialog.h
  include/rqt_multiplot/CurveConfigWidget.h
  include/rqt_multiplot/CurveData.h
  include/rqt_multiplot/CurveDataCircularBuffer.h
  include/rqt_multiplot/CurveDataConfig.h
  include/rqt_multiplot/CurveDataConfigWidget.h
  include/rqt_multiplot/CurveDataList.h
  include/rqt_multiplot/CurveDataSequencer.h
  include/rqt_multiplot/CurveDataVector.h
  include/rqt_multiplot/CurveItemWidget.h
  include/rqt_multiplot/CurveListWidget.h
  include/rqt_multiplot/CurveStyle.h
  include/rqt_multiplot/CurveStyleWidget.h
  include/rqt_multiplot/FileScheme.h
  include/rqt_multiplot/Message.h
  include/rqt_multiplot/MessageDefinitionLoader.h
  include/rqt_multiplot/MessageFieldCompleter.h
  include/rqt_multiplot/MessageFieldItem.h
  include/rqt_multiplot/MessageFieldItemModel.h
  include/rqt_multiplot/MessageFieldLineEdit.h
  include/rqt_multiplot/MessageFieldSubscriber.h
  include/rqt_multiplot/MessageFieldSubscriberRegistry.h
  include/rqt_multiplot/MessageFieldTreeWidget.h
  include/rqt_multiplot/MessageFieldWidget.h
  include/rqt_multiplot/MessageSubscriber.h
  include/rqt_multiplot/MessageSubscriberRegistry.h
  include/rqt_multiplot/MessageTopicComboBox.h
  include/rqt_multiplot/MessageTopicRegistry.h
  include/rqt_multiplot/MessageTypeComboBox.h
  include/rqt_multiplot/MessageTypeRegistry.h
  include/rqt_multiplot/MultiplotConfig.h
  include/rqt_multiplot/MultiplotConfigWidget.h
  include/rqt_multiplot/MultiplotPlugin.h
  include/rqt_multiplot/MultiplotWidget.h
  include/rqt_multiplot/PackageRegistry.h
  include/rqt_multiplot/PackageScheme.h
  include/rqt_multiplot/PenStyleComboBox.h
  include/rqt_multiplot/PenStyleItemDelegate.h
  include/rqt_multiplot/PlotAxesConfig.h
  include/rqt_multiplot/PlotAxesConfigWidget.h
  include/rqt_multiplot/PlotAxisConfig.h
  include/rqt_multiplot/PlotAxisConfigWidget.h
  include/rqt_multiplot/PlotConfig.h
  include/rqt_multiplot/PlotConfigDialog.h
  include/rqt_multiplot/PlotConfigWidget.h
  include/rqt_multiplot/PlotCursor.h
  include/rqt_multiplot/PlotCursorMachine.h
  include/rqt_multiplot/PlotCurve.h
  include/rqt_multiplot/PlotLegend.h
  include/rqt_multiplot/PlotLegendConfig.h
  include/rqt_multiplot/PlotLegendConfigWidget.h
  include/rqt_multiplot/PlotMagnifier.h
  include/rqt_multiplot/PlotPanner.h
  include/rqt_multiplot/PlotTableConfig.h
  include/rqt_multiplot/PlotTableConfigWidget.h
  include/rqt_multiplot/PlotTableWidget.h
  include/rqt_multiplot/PlotWidget.h
  include/rqt_multiplot/PlotZoomer.h
  include/rqt_multiplot/PlotZoomerMachine.h
  include/rqt_multiplot/StatusWidget.h
  include/rqt_multiplot/UrlComboBox.h
  include/rqt_multiplot/UrlCompleter.h
  include/rqt_multiplot/UrlItem.h
  include/rqt_multiplot/UrlItemModel.h
  include/rqt_multiplot/UrlScheme.h
  include/rqt_multiplot/XmlSettings.h
)

set(rqt_multiplot_UIS
  src/CurveAxisConfigWidget.ui
  src/CurveAxisScaleWidget.ui
  src/CurveColorWidget.ui
  src/CurveConfigDialog.ui
  src/CurveConfigWidget.ui
  src/CurveDataConfigWidget.ui
  src/CurveItemWidget.ui
  src/CurveStyleWidget.ui
  src/MessageFieldWidget.ui
  src/MultiplotConfigWidget.ui
  src/MultiplotWidget.ui
  src/PlotAxesConfigWidget.ui
  src/PlotAxisConfigWidget.ui
  src/PlotConfigDialog.ui
  src/PlotConfigWidget.ui
  src/PlotLegendConfigWidget.ui
  src/PlotTableConfigWidget.ui
  src/PlotWidget.ui
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
