cmake_minimum_required(VERSION 2.8)
project(quadprog CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


set(CMAKE_DEBUG_POSTFIX "_d")

if(MSVC)
    SET(BUILD_SHARED_LIBS OFF)
    set(suffix_type "_s")
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

# Check dependencies
find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(PC_EIGEN eigen3<=3.0.5 REQUIRED)
if(NOT PC_EIGEN_FOUND)
    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
endif(NOT PC_EIGEN_FOUND)

#set installation
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set(CMAKE_INSTALL_PREFIX /usr/local/)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

install(DIRECTORY include/ DESTINATION include/ )


add_subdirectory(pkg-config)
add_subdirectory(include)
add_subdirectory(src)
