cmake_minimum_required(VERSION 3.10)

project(t265_standalone_debug LANGUAGES CXX)

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
    message(FATAL_ERROR "librealsense 2.50 library not found: ${REALSENSE_LIBRARY}")
endif()

add_executable(t265_debug src/t265_debug.cpp)
target_include_directories(t265_debug PRIVATE "${REALSENSE_INCLUDE_DIR}")
target_link_libraries(t265_debug PRIVATE "${REALSENSE_LIBRARY}" pthread)

find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc)
add_executable(t265_trajectory_debug src/t265_trajectory_debug.cpp)
target_include_directories(t265_trajectory_debug PRIVATE "${REALSENSE_INCLUDE_DIR}")
target_link_libraries(t265_trajectory_debug PRIVATE "${REALSENSE_LIBRARY}" pthread ${OpenCV_LIBS})

# Keep this utility bound to the locally built official 2.50 library, even if
# another librealsense version is installed system-wide.
set_target_properties(t265_debug PROPERTIES
    BUILD_RPATH "${REALSENSE_ROOT}/build-250/librealsense"
    INSTALL_RPATH "${REALSENSE_ROOT}/build-250/librealsense")
set_target_properties(t265_trajectory_debug PROPERTIES
    BUILD_RPATH "${REALSENSE_ROOT}/build-250/librealsense"
    INSTALL_RPATH "${REALSENSE_ROOT}/build-250/librealsense")

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

set(T265_FIRMWARE
    "${REALSENSE_ROOT}/build-250/librealsense/common/fw/target-0.2.0.951.mvcmd")
if(NOT EXISTS "${T265_FIRMWARE}")
    message(FATAL_ERROR "T265 firmware not found: ${T265_FIRMWARE}")
endif()

add_executable(t265_boot src/t265_boot.cpp)
target_link_libraries(t265_boot PRIVATE PkgConfig::LIBUSB)
target_compile_definitions(t265_boot PRIVATE T265_FIRMWARE_PATH="${T265_FIRMWARE}")
