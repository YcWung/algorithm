cmake_minimum_required(VERSION 3.21)

project(algorithm VERSION 0.0.1 LANGUAGES CXX)

#=================================== options ==================================#

option(BUILD_SHARED_LIBS "Build shared library" ON)
option(BUILD_TESTS "Build unit tests" ON)

#================================= file/dir names =============================#

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_DEBUG_POSTFIX d)

#===================================== CXX ====================================#

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    add_compile_options(/W3 /WX /bigobj /MP8)
endif()

#--------------------------------- find packages ------------------------------#

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

#================================== add subdirectories ========================#

add_library(algorithm INTERFACE)
target_include_directories(algorithm INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
