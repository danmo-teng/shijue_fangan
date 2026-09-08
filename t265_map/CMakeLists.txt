cmake_minimum_required(VERSION 3.10)

project(t265_map_builder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The RDK repository intentionally does not vendor librealsense. Keep this
# utility tied to the same locally built official 2.50 library as the main
# localization executable, while still allowing another checkout to pass
# -DREALSENSE_ROOT=/path/to/T265.
set(REALSENSE_ROOT "/home/sunrise/文档/ChatGPT/T265" CACHE PATH
    "Directory containing third_party/librealsense and build-250")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_ROOT}/third_party/librealsense/include")
set(REALSENSE_LIBRARY "${REALSENSE_ROOT}/build-250/librealsense/librealsense2.so.2.50.0")

if(NOT EXISTS "${REALSENSE_INCLUDE_DIR}/librealsense2/rs.hpp")
    message(FATAL_ERROR "librealsense headers not found: ${REALSENSE_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${REALSENSE_LIBRARY}")
    message(FATAL_ERROR "official librealsense 2.50 library not found: ${REALSENSE_LIBRARY}")
endif()

find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc)
find_package(X11 REQUIRED)

set(LOCALIZATION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../localization")
add_library(t265_map_localization_core STATIC
    "${LOCALIZATION_ROOT}/src/config.cpp"
    "${LOCALIZATION_ROOT}/src/fusion.cpp"
    "${LOCALIZATION_ROOT}/src/serial_port.cpp")
target_include_directories(t265_map_localization_core PUBLIC
    "${LOCALIZATION_ROOT}/include")
target_link_libraries(t265_map_localization_core PUBLIC pthread)

add_executable(t265_map_builder
    src/t265_map_builder.cpp
    src/scan_protocol.cpp)
target_include_directories(t265_map_builder PRIVATE
    include
    "${LOCALIZATION_ROOT}/include"
    "${REALSENSE_INCLUDE_DIR}"
    ${X11_INCLUDE_DIR})
target_link_libraries(t265_map_builder PRIVATE
    t265_map_localization_core
    "${REALSENSE_LIBRARY}"
    pthread
    ${X11_LIBRARIES}
    ${OpenCV_LIBS})
target_compile_definitions(t265_map_builder PRIVATE
    T265_MAP_DEFAULT_CONFIG="${LOCALIZATION_ROOT}/config/localization.example.conf"
    T265_MAP_DEFAULT_SESSION_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../rescue_map/runtime/history/t265_map_builder")
set_target_properties(t265_map_builder PROPERTIES
    BUILD_RPATH "${REALSENSE_ROOT}/build-250/librealsense"
    INSTALL_RPATH "${REALSENSE_ROOT}/build-250/librealsense")

option(BUILD_TESTING "Build scanner protocol tests" ON)
if(BUILD_TESTING)
    enable_testing()
    add_executable(test_scan_protocol
        tests/test_scan_protocol.cpp
        src/scan_protocol.cpp)
    target_include_directories(test_scan_protocol PRIVATE include)
    add_test(NAME scan_protocol_tests COMMAND test_scan_protocol)
endif()
