cmake_minimum_required(VERSION 3.10)

project(t265_omni_localization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(REALSENSE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH
    "Directory containing third_party/librealsense and build-250")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_ROOT}/third_party/librealsense/include")
set(REALSENSE_LIBRARY "${REALSENSE_ROOT}/build-250/librealsense/librealsense2.so.2.50.0")

if(NOT EXISTS "${REALSENSE_INCLUDE_DIR}/librealsense2/rs.hpp")
    message(FATAL_ERROR "librealsense headers not found: ${REALSENSE_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${REALSENSE_LIBRARY}")
    message(FATAL_ERROR "official librealsense 2.50 library not found: ${REALSENSE_LIBRARY}")
endif()
add_library(omni_localization_core
    src/config.cpp
    src/f407_protocol.cpp
    src/fusion.cpp
    src/serial_port.cpp)
target_include_directories(omni_localization_core PUBLIC include)
target_link_libraries(omni_localization_core PUBLIC pthread)

add_executable(t265_omni_localizer src/main.cpp)
target_include_directories(t265_omni_localizer PRIVATE "${REALSENSE_INCLUDE_DIR}")
target_link_libraries(t265_omni_localizer PRIVATE
    omni_localization_core "${REALSENSE_LIBRARY}")
set_target_properties(t265_omni_localizer PROPERTIES
    BUILD_RPATH "${REALSENSE_ROOT}/build-250/librealsense"
    INSTALL_RPATH "${REALSENSE_ROOT}/build-250/librealsense")

option(BUILD_TESTING "Build protocol and fusion tests" ON)
if(BUILD_TESTING)
    enable_testing()
    add_executable(test_omni_localization tests/test_omni_localization.cpp)
    target_link_libraries(test_omni_localization PRIVATE omni_localization_core)
    add_test(NAME omni_localization_tests COMMAND test_omni_localization)
endif()
