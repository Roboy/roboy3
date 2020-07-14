cmake_minimum_required(VERSION 2.8.3)
project( roboy_motor_calibration)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS roscpp roslint rqt_gui rqt_gui_cpp std_msgs
        roboy_middleware_msgs common_utilities )

include_directories( include qcustomplot)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_middleware_msgs common_utilities)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(roboy_motor_calibration_MOCS include/roboy_motor_calibration/roboy_motor_calibration.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(roboy_motor_calibration_UIS_H src/roboy_motor_calibration.ui)
else()
  qt4_wrap_cpp(roboy_motor_calibration_MOCS include/roboy_motor_calibration/roboy_motor_calibration.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(roboy_motor_calibration_UIS_H src/roboy_motor_calibration.ui)
endif()

include_directories(
        include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library( roboy_motor_calibration src/roboy_motor_calibration.cpp ${roboy_motor_calibration_MOCS} ${roboy_motor_calibration_UIS_H}
        qcustomplot/qcustomplot.cpp)

add_dependencies(roboy_motor_calibration roboy_middleware_msgs_gencpp)

target_link_libraries(roboy_motor_calibration ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(roboy_motor_calibration Qt5::Widgets)
else()
  target_link_libraries(roboy_motor_calibration ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS roboy_motor_calibration
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/roboy_rqt_plugins
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )