cmake_minimum_required(VERSION 3.5)
project(a_loam_mt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV 3.4 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include /usr/include/eigen3 ${PCL_INCLUDE_DIRS})
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)

add_library(a_loam_mt ${SRC_FILES})

target_link_libraries(
    a_loam_mt
    ${PCL_LIBRARIES}
    Sophus::Sophus
    g2o_core
    g2o_stuff
    TBB::tbb
    ${OpenCV_LIBS})
ament_target_dependencies(a_loam_mt rclcpp)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(test)

ament_package()
