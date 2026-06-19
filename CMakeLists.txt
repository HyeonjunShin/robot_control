cmake_minimum_required(VERSION 3.25)
project(robot_control LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include(FetchContent)
set(MUJOCO_BUILD_EXAMPLES OFF CACHE INTERNAL "")
set(MUJOCO_BUILD_SIMULATE OFF CACHE INTERNAL "")
set(MUJOCO_BUILD_TESTS OFF CACHE INTERNAL "")
FetchContent_Declare(
  mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG        main 
)
FetchContent_MakeAvailable(mujoco)


add_executable(gui examples/gui.cpp)
find_package(zenohc REQUIRED) 
find_package(zenohcxx REQUIRED)

target_link_libraries(gui PRIVATE 
  mujoco::mujoco          
  glfw                    
  zenohcxx::zenohc
)