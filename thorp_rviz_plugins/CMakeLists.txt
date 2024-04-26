cmake_minimum_required(VERSION 3.10)
project(thorp_rviz_plugins)

set(PKG_DEPS
  geometry_msgs
  std_srvs
  pluginlib
  roscpp
  jsk_rviz_plugins
  rviz
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets Qt5::Core)

find_package(OGRE QUIET COMPONENTS Overlay)
if(OGRE_FOUND)
  if(${OGRE_VERSION} VERSION_LESS 1.11)
    set(OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} ${OGRE_Overlay_INCLUDE_DIRS})
    set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${OGRE_Overlay_LIBRARIES})
  endif()
else()
  # OGRE doesn't come with correctly installed cmake files on Linux. Use pkg-config instead.
  # We need both, OGRE and OGRE-Overlay. Look for both simulatenously and use prefix X_
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(X REQUIRED OGRE OGRE-Overlay)

  # Set OGRE_* variables as cmake-based find_package would do
  set(OGRE_VERSION ${X_OGRE_VERSION})
  set(OGRE_INCLUDE_DIRS ${X_INCLUDE_DIRS})
  message(STATUS "OGRE_INCLUDE_DIRS=${OGRE_INCLUDE_DIRS}")

  # Find absolute path of OGRE libraries.
  # This is stored in the cache to save time on cmake re-run.
  # Using absolute paths is necessary if pkg-config finds Ogre in a different
  # location than the default.  This can happen when Ogre is built from source,
  # or when 2 versions of Ogre are installed.  Using absolute paths ensures that
  # components that link against Ogre (rviz and any packages depending on rviz)
  # all find the same Ogre shared library.
  if(NOT DEFINED OGRE_LIBRARIES)
    set(_OGRE_LIBRARIES)
    foreach(_lib ${X_LIBRARIES})
      set(_lib_tag "OGRE_RVIZ_${_lib}")

      find_library(${_lib_tag}
              NAMES ${_lib}
              HINTS ${X_LIBRARY_DIRS}
              PATHS ${X_LIBRARY_DIRS}
      )

      list(APPEND _OGRE_LIBRARIES ${${_lib_tag}})
    endforeach(_lib)

    set(OGRE_LIBRARIES ${_OGRE_LIBRARIES} CACHE STRING "Absolute paths to OGRE libs")
  endif(NOT DEFINED OGRE_LIBRARIES)
  message(STATUS "OGRE_LIBRARIES=${OGRE_LIBRARIES}")

  ## Fetch OGRE_PLUGIN_DIR variable from pkg-config
  if(NOT DEFINED OGRE_PLUGIN_DIR)
    execute_process(COMMAND
      ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
      OUTPUT_VARIABLE OGRE_PLUGIN_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
  endif(NOT DEFINED OGRE_PLUGIN_DIR)
  message(STATUS "OGRE_PLUGIN_DIR=${OGRE_PLUGIN_DIR}")
endif()

# Disable deprecation warnings for OGRE >= 1.10
if(NOT OGRE_VERSION VERSION_LESS "1.10.0" AND NOT MSVC)
  add_compile_options(-Wno-deprecated-declarations)
endif()

set(LIBS
  ${PROJECT_NAME}_button_tool
  ${PROJECT_NAME}_cancel_navigation_tool
  ${PROJECT_NAME}_clear_costmap_tool
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${OGRE_LIBRARIES}
    ${LIBS}
  CATKIN_DEPENDS ${PKG_DEPS}
)

#############
##  Build  ##
#############
include_directories(
  include
  SYSTEM ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

# Avoid keyword definition to avoid conflicts with boost or xapian etc
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
add_definitions(-DQT_NO_KEYWORDS)

# button tool
add_library(${PROJECT_NAME}_button_tool
        src/button_tool.cpp
)
target_link_libraries(${PROJECT_NAME}_button_tool
        ${catkin_LIBRARIES}
        ${QT_LIBRARIES}
)

# Cancel navigation tool
qt5_wrap_cpp(${PROJECT_NAME}_cancel_navigation_tool_MOCS include/${PROJECT_NAME}/cancel_navigation_tool.hpp)

add_library(${PROJECT_NAME}_cancel_navigation_tool
  src/cancel_navigation_tool.cpp
  ${${PROJECT_NAME}_cancel_navigation_tool_MOCS}
)
target_link_libraries(${PROJECT_NAME}_cancel_navigation_tool
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${PROJECT_NAME}_button_tool
)

# Clear costmap tool
qt5_wrap_cpp(${PROJECT_NAME}_clear_costmap_tool_MOCS include/${PROJECT_NAME}/clear_costmap_tool.hpp)

add_library(${PROJECT_NAME}_clear_costmap_tool
  src/clear_costmap_tool.cpp
  ${${PROJECT_NAME}_clear_costmap_tool_MOCS}
)
target_link_libraries(${PROJECT_NAME}_clear_costmap_tool
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${PROJECT_NAME}_button_tool
)

#############
## Install ##
#############
install(TARGETS
  ${LIBS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
