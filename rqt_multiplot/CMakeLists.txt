cmake_minimum_required(VERSION 2.8.3)
project(rqt_multiplot)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    rosbag
    roscpp
    rqt_gui
    rqt_gui_cpp
    variant_topic_tools
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

find_path(
  QWT_INCLUDE_DIRS
  NAMES
    qwt_plot.h
  PATHS
    /usr/include/qwt-qt4
    /usr/include/qwt
    /usr/local/include/qwt
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_library(
    QWT_LIBRARIES
    NAMES
      qwt-qt5
  )
else()
  find_library(
    QWT_LIBRARIES
    NAMES
      qwt-qt4
      qwt
  )
endif()

catkin_python_setup()

set(rqt_multiplot_SRCS
  src/rqt_multiplot/BagQuery.cpp
  src/rqt_multiplot/BagReader.cpp
  src/rqt_multiplot/BitOperations.cpp
  src/rqt_multiplot/BoundingRectangle.cpp
  src/rqt_multiplot/ColorOperations.cpp
  src/rqt_multiplot/Config.cpp
  src/rqt_multiplot/ConfigComboBox.cpp
  src/rqt_multiplot/CurveAxisConfig.cpp
  src/rqt_multiplot/CurveAxisConfigWidget.cpp
  src/rqt_multiplot/CurveAxisScaleConfig.cpp
  src/rqt_multiplot/CurveAxisScaleConfigWidget.cpp
  src/rqt_multiplot/CurveColorConfig.cpp
  src/rqt_multiplot/CurveColorConfigWidget.cpp
  src/rqt_multiplot/CurveConfig.cpp
  src/rqt_multiplot/CurveConfigDialog.cpp
  src/rqt_multiplot/CurveConfigWidget.cpp
  src/rqt_multiplot/CurveData.cpp
  src/rqt_multiplot/CurveDataCircularBuffer.cpp
  src/rqt_multiplot/CurveDataConfig.cpp
  src/rqt_multiplot/CurveDataConfigWidget.cpp
  src/rqt_multiplot/CurveDataList.cpp
  src/rqt_multiplot/CurveDataListTimeFrame.cpp
  src/rqt_multiplot/CurveDataSequencer.cpp
  src/rqt_multiplot/CurveDataVector.cpp
  src/rqt_multiplot/CurveItemWidget.cpp
  src/rqt_multiplot/CurveListWidget.cpp
  src/rqt_multiplot/CurveStyleConfig.cpp
  src/rqt_multiplot/CurveStyleConfigWidget.cpp
  src/rqt_multiplot/DataTypeRegistry.cpp
  src/rqt_multiplot/FileScheme.cpp
  src/rqt_multiplot/MatchFilterComboBox.cpp
  src/rqt_multiplot/MatchFilterCompleter.cpp
  src/rqt_multiplot/MatchFilterCompleterModel.cpp
  src/rqt_multiplot/Message.cpp
  src/rqt_multiplot/MessageBroker.cpp
  src/rqt_multiplot/MessageDefinitionLoader.cpp
  src/rqt_multiplot/MessageEvent.cpp
  src/rqt_multiplot/MessageFieldCompleter.cpp
  src/rqt_multiplot/MessageFieldItem.cpp
  src/rqt_multiplot/MessageFieldItemModel.cpp
  src/rqt_multiplot/MessageFieldLineEdit.cpp
  src/rqt_multiplot/MessageFieldTreeWidget.cpp
  src/rqt_multiplot/MessageFieldWidget.cpp
  src/rqt_multiplot/MessageSubscriber.cpp
  src/rqt_multiplot/MessageSubscriberRegistry.cpp
  src/rqt_multiplot/MessageTopicComboBox.cpp
  src/rqt_multiplot/MessageTopicRegistry.cpp
  src/rqt_multiplot/MessageTypeComboBox.cpp
  src/rqt_multiplot/MessageTypeRegistry.cpp
  src/rqt_multiplot/MultiplotConfig.cpp
  src/rqt_multiplot/MultiplotConfigWidget.cpp
  src/rqt_multiplot/MultiplotPlugin.cpp
  src/rqt_multiplot/MultiplotWidget.cpp
  src/rqt_multiplot/PackageRegistry.cpp
  src/rqt_multiplot/PackageScheme.cpp
  src/rqt_multiplot/PenStyleComboBox.cpp
  src/rqt_multiplot/PenStyleItemDelegate.cpp
  src/rqt_multiplot/PlotAxesConfig.cpp
  src/rqt_multiplot/PlotAxesConfigWidget.cpp
  src/rqt_multiplot/PlotAxisConfig.cpp
  src/rqt_multiplot/PlotAxisConfigWidget.cpp
  src/rqt_multiplot/PlotConfig.cpp
  src/rqt_multiplot/PlotConfigDialog.cpp
  src/rqt_multiplot/PlotConfigWidget.cpp
  src/rqt_multiplot/PlotCursor.cpp
  src/rqt_multiplot/PlotCursorMachine.cpp
  src/rqt_multiplot/PlotCurve.cpp
  src/rqt_multiplot/PlotLegend.cpp
  src/rqt_multiplot/PlotLegendConfig.cpp
  src/rqt_multiplot/PlotLegendConfigWidget.cpp
  src/rqt_multiplot/PlotMagnifier.cpp
  src/rqt_multiplot/PlotPanner.cpp
  src/rqt_multiplot/PlotTableConfig.cpp
  src/rqt_multiplot/PlotTableConfigWidget.cpp
  src/rqt_multiplot/PlotTableWidget.cpp
  src/rqt_multiplot/PlotWidget.cpp
  src/rqt_multiplot/PlotZoomer.cpp
  src/rqt_multiplot/PlotZoomerMachine.cpp
  src/rqt_multiplot/ProgressChangeEvent.cpp
  src/rqt_multiplot/ProgressWidget.cpp
  src/rqt_multiplot/QwtPlotCustom.cpp
  src/rqt_multiplot/StatusWidget.cpp
  src/rqt_multiplot/ThreadedTimer.cpp
  src/rqt_multiplot/UrlComboBox.cpp
  src/rqt_multiplot/UrlCompleter.cpp
  src/rqt_multiplot/UrlItem.cpp
  src/rqt_multiplot/UrlItemModel.cpp
  src/rqt_multiplot/UrlScheme.cpp
  src/rqt_multiplot/XmlSettings.cpp
)

set(rqt_multiplot_HDRS
  include/rqt_multiplot/BagQuery.h
  include/rqt_multiplot/BagReader.h
  include/rqt_multiplot/BitOperations.h
  include/rqt_multiplot/BoundingRectangle.h
  include/rqt_multiplot/ColorOperations.h
  include/rqt_multiplot/Config.h
  include/rqt_multiplot/ConfigComboBox.h
  include/rqt_multiplot/CurveAxisConfig.h
  include/rqt_multiplot/CurveAxisConfigWidget.h
  include/rqt_multiplot/CurveAxisScaleConfig.h
  include/rqt_multiplot/CurveAxisScaleConfigWidget.h
  include/rqt_multiplot/CurveColorConfig.h
  include/rqt_multiplot/CurveColorConfigWidget.h
  include/rqt_multiplot/CurveConfig.h
  include/rqt_multiplot/CurveConfigDialog.h
  include/rqt_multiplot/CurveConfigWidget.h
  include/rqt_multiplot/CurveData.h
  include/rqt_multiplot/CurveDataCircularBuffer.h
  include/rqt_multiplot/CurveDataConfig.h
  include/rqt_multiplot/CurveDataConfigWidget.h
  include/rqt_multiplot/CurveDataList.h
  include/rqt_multiplot/CurveDataListTimeFrame.h
  include/rqt_multiplot/CurveDataSequencer.h
  include/rqt_multiplot/CurveDataVector.h
  include/rqt_multiplot/CurveItemWidget.h
  include/rqt_multiplot/CurveListWidget.h
  include/rqt_multiplot/CurveStyleConfig.h
  include/rqt_multiplot/CurveStyleConfigWidget.h
  include/rqt_multiplot/DataTypeRegistry.h
  include/rqt_multiplot/FileScheme.h
  include/rqt_multiplot/MatchFilterComboBox.h
  include/rqt_multiplot/MatchFilterCompleter.h
  include/rqt_multiplot/MatchFilterCompleterModel.h
  include/rqt_multiplot/Message.h
  include/rqt_multiplot/MessageBroker.h
  include/rqt_multiplot/MessageDefinitionLoader.h
  include/rqt_multiplot/MessageEvent.h
  include/rqt_multiplot/MessageFieldCompleter.h
  include/rqt_multiplot/MessageFieldItem.h
  include/rqt_multiplot/MessageFieldItemModel.h
  include/rqt_multiplot/MessageFieldLineEdit.h
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
  include/rqt_multiplot/ProgressChangeEvent.h
  include/rqt_multiplot/ProgressWidget.h
  include/rqt_multiplot/QwtPlotCustom.h
  include/rqt_multiplot/StatusWidget.h
  include/rqt_multiplot/ThreadedTimer.h
  include/rqt_multiplot/UrlComboBox.h
  include/rqt_multiplot/UrlCompleter.h
  include/rqt_multiplot/UrlItem.h
  include/rqt_multiplot/UrlItemModel.h
  include/rqt_multiplot/UrlScheme.h
  include/rqt_multiplot/XmlSettings.h
)

set(rqt_multiplot_UIS
  src/rqt_multiplot/CurveAxisConfigWidget.ui
  src/rqt_multiplot/CurveAxisScaleConfigWidget.ui
  src/rqt_multiplot/CurveColorConfigWidget.ui
  src/rqt_multiplot/CurveConfigDialog.ui
  src/rqt_multiplot/CurveConfigWidget.ui
  src/rqt_multiplot/CurveDataConfigWidget.ui
  src/rqt_multiplot/CurveItemWidget.ui
  src/rqt_multiplot/CurveStyleConfigWidget.ui
  src/rqt_multiplot/MessageFieldWidget.ui
  src/rqt_multiplot/MultiplotConfigWidget.ui
  src/rqt_multiplot/MultiplotWidget.ui
  src/rqt_multiplot/PlotAxesConfigWidget.ui
  src/rqt_multiplot/PlotAxisConfigWidget.ui
  src/rqt_multiplot/PlotConfigDialog.ui
  src/rqt_multiplot/PlotConfigWidget.ui
  src/rqt_multiplot/PlotLegendConfigWidget.ui
  src/rqt_multiplot/PlotTableConfigWidget.ui
  src/rqt_multiplot/PlotWidget.ui
  src/rqt_multiplot/ProgressWidget.ui
)

set(rqt_multiplot_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  LIBRARIES
    rqt_multiplot
  CATKIN_DEPENDS
    rosbag
    roscpp
    rqt_gui
    rqt_gui_cpp
    variant_topic_tools
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_multiplot_MOCS ${rqt_multiplot_HDRS})
  qt5_wrap_ui(rqt_multiplot_UIS_H ${rqt_multiplot_UIS})
else()
  qt4_wrap_cpp(rqt_multiplot_MOCS ${rqt_multiplot_HDRS})
  qt4_wrap_ui(rqt_multiplot_UIS_H ${rqt_multiplot_UIS})
endif()

include_directories(
  ${rqt_multiplot_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${QWT_INCLUDE_DIRS}
)

add_library(rqt_multiplot
  ${rqt_multiplot_SRCS}
  ${rqt_multiplot_MOCS}
  ${rqt_multiplot_UIS_H}
)

target_link_libraries(rqt_multiplot
  ${catkin_LIBRARIES}
  ${QWT_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_use_modules(${PROJECT_NAME} Widgets)
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(rqt_multiplot)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.png"
)

install(TARGETS rqt_multiplot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_multiplot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
