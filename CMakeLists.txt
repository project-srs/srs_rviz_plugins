cmake_minimum_required(VERSION 3.8)
project(srs_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(cv_bridge REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  # joy panel
  src/joy_panel/joy_panel.cpp
  src/joy_panel/touch_widget.cpp
  # string panel
  src/string_panel/string_panel.cpp
  src/string_panel/string_panel_qt_if.cpp
  # string display
  src/string_display/string_display.cpp
  # color_display
  src/color_display/color_display.cpp
  # image_quad_display
  src/image_quad_display/image_quad_display.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  std_msgs
  rviz_common
  rviz_rendering
  cv_bridge
)
install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
