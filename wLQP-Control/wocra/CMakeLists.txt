cmake_minimum_required(VERSION 2.8)
project(wLQP-Control CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_CXX_FLAGS "-fpermissive")

if(MSVC)
    SET(BUILD_SHARED_LIBS OFF)
    set(suffix_type "_static")
    set(install_type "ARCHIVE")
else()
    SET(BUILD_SHARED_LIBS ON)
    set(suffix_type "")
    if(UNIX)
        set(install_type "LIBRARY")
    else()
        set(install_type "RUNTIME")
    endif()
endif()


##### DEFINE FUNCTION TO GET REQUIRED LIB WITH SAME BUILD TYPE IF POSSIBLE #####
function(get_package_with_same_build_type PACKAGE_ALIAS package_name)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
    PKG_CHECK_MODULES(${PACKAGE_ALIAS} ${package_name}_dbg)
    if(NOT ${PACKAGE_ALIAS}_FOUND)
        message("Cannot detect debug version of ${package_name}, check release version..." )
        PKG_CHECK_MODULES(${PACKAGE_ALIAS} ${package_name} REQUIRED)
    endif()
ELSE()
        PKG_CHECK_MODULES(${PACKAGE_ALIAS} ${package_name} REQUIRED)
ENDIF()

if(NOT ${PACKAGE_ALIAS}_FOUND)
    message("Failed to detect ${package_name}, check PKG_CONFIG_PATH." )
endif()

endfunction()


# Check dependencies
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED)
PKG_CHECK_MODULES(PC_EIGEN eigen3<=3.0.5 REQUIRED)
if(NOT PC_EIGEN_FOUND)
    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
endif(NOT PC_EIGEN_FOUND)

get_package_with_same_build_type(PC_QUADPROG quadprog)
get_package_with_same_build_type(PC_OCRA_FRAMEWORK ocra_framework)

add_subdirectory(pkg-config)
add_subdirectory(include)
add_subdirectory(src)