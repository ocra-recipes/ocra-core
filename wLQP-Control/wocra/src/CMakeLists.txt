include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
    ${ocra_framework_SOURCE_DIR}/include
    ${quadprog_SOURCE_DIR}/include
    ${PC_EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${YARP_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
    ${ocra_framework_SOURCE_DIR}/src
    ${quadprog_SOURCE_DIR}/src
)

file(
	GLOB_RECURSE
	source_files

	*.cpp
)


# Check for SMLT
find_package(SMLT)
IF(SMLT_FOUND)
    message("-- SMLT lib found. Compiling gaussian process trajectory.")
    include_directories(SMLT_INCLUDE_DIRS)
ELSE()
    message("-- SMLT lib NOT found. Removing gaussian process trajectory from source list.")
    list(REMOVE_ITEM source_files ${CMAKE_CURRENT_SOURCE_DIR}/src/Trajectory/wOcraGaussianProcessTrajectory.cpp)
ENDIF()


add_library(
	wLQP-Control${suffix_type}
	${LIB_TYPE}
	${source_files}
)

target_link_libraries(
    wLQP-Control
    ocra_control
    ocra_optim
    quadprog
    ${YARP_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${Boost_LIBRARIES}
    ${SMLT_LIBRARIES}

)

install(
    TARGETS wLQP-Control
    ${install_type} DESTINATION lib
)
