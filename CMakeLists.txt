##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(topic_tracker)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp topic_tools std_msgs)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Multimedia Network)

include_directories(${catkin_INCLUDE_DIRS}
                    include/topic_tracker)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

set(QT_LIBRARIES Qt5::Widgets Qt5::Multimedia Qt5::Core Qt5::Network Qt5::Gui)
set(QTVERSION ${Qt5Widgets_VERSION})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/topic_tracker/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

set(QT_SOURCES src/main.cpp 
               src/qnode.cpp 
               src/main_window.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(topic_tracker ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(topic_tracker ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS topic_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

