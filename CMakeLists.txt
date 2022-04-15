cmake_minimum_required(VERSION 3.8)
project(hik_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hik_camera_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC hikSDK/include)
target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib)
target_link_libraries(${PROJECT_NAME} FormatConversion MediaProcess MvCameraControl MVRender MvUsb3vTL
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  set(_linter_excludes
    hikSDK/include/CameraParams.h
    hikSDK/include/MvCameraControl.h
    hikSDK/include/MvErrorDefine.h
    hikSDK/include/MvISPErrorDefine.h
    hikSDK/include/PixelType.h
  )
  ament_cpplint(EXCLUDE ${_linter_excludes})

endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
