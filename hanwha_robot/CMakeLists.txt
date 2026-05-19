cmake_minimum_required(VERSION 3.25)

# 정의, 현재 처리중인 디렉토리를 기준으로 LIB 파일들의 경로 지정
set(HANWHA_ROBOT_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/lib")
set(HANWHA_ROBOT_KIN_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/kinematics")

# 라이브러리 정의
add_library(hanwha_robot STATIC ${CMAKE_CURRENT_LIST_DIR}/src/hanwha_robot.cpp)

# 헤더 링킹
target_include_directories(hanwha_robot PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

# 라이브러리 링킹
target_link_libraries(hanwha_robot PUBLIC
        "${HANWHA_ROBOT_LIB_DIR}/libclink_api_rpc_client.so"
        "${HANWHA_ROBOT_KIN_DIR}/libkinematics_hcr_3gen.so")

# 링크 옵션
target_link_options(hanwha_robot PUBLIC
        "-Wl,-rpath,${HANWHA_ROBOT_LIB_DIR}"
        "-Wl,-rpath,${HANWHA_ROBOT_KIN_DIR}"
        "-Wl,-rpath-link,${HANWHA_ROBOT_LIB_DIR}"
        "-Wl,-rpath-link,${HANWHA_ROBOT_KIN_DIR}")

# 매크로 상수로 접근이 가능하도록 정의
target_compile_definitions(hanwha_robot PUBLIC
        HANWHA_ROOT_PATH="${CMAKE_CURRENT_LIST_DIR}")

