cmake_minimum_required(VERSION 2.8)


LIST(APPEND PC_EIGEN_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/eigen_3_0_5/" "${CMAKE_CURRENT_SOURCE_DIR}/eigen_3_0_5/unsupported/")

add_subdirectory(core-framework)
add_subdirectory(wLQP-Control)
