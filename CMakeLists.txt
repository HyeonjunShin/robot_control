cmake_minimum_required(VERSION 3.25)
project(robot_control)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

FetchContent_Declare(
  mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG        main # 또는 특정 버전 (예: CMake에서 지원하는 최신 태그 v3.x.x)
)
FetchContent_MakeAvailable(mujoco)

add_executable(gui examples/gui.cpp)
target_link_libraries(gui PRIVATE 
  mujoco::mujoco
  glfw
)
# add_subdirectory(hanwha_robot)

# add_library(robot_control STATIC
        # ${CMAKE_CURRENT_LIST_DIR}/src/robot_control.cpp)

# target_include_directories(robot_control PUBLIC
        # ${CMAKE_CURRENT_LIST_DIR}/include)

# add_exutable(main ${CMAKE_CURRENT_LIST_DIR}/examples/robot_control.cpp)
