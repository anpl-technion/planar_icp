cmake_minimum_required(VERSION 2.6)
enable_testing()
project(planar_icp CXX C)

# Frank: I seem to have to do this pretty early or it does not stick
set(GTSAM_TOOLBOX_INSTALL_PATH "${PROJECT_SOURCE_DIR}/matlab/planar_icp" CACHE PATH "Install folder for matlab toolbox - defaults to inside project")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/cmake")
include(GtsamMakeConfigFile)

# Load build type flags and default to Debug mode
include(GtsamBuildTypes)

# Include gtsam
find_package(GTSAM REQUIRED NO_MODULE) # Uses installed package
set(gtsam_deps gtsam)

########################## Andrea Censi's CSM #######################################
# CSM Code available here: https://github.com/AndreaCensi/csm
# It will also be necessary to have the GNU Scientific Library (libgsl) installed as a dependency
# Require we have pkgconfig installed
find_package(PkgConfig REQUIRED)

# Tell pkgconfig to look for CSM
pkg_check_modules(CSM REQUIRED csm)

IF(${CSM_FOUND})
	# Remove unecessary include of the the path "."
	# Linker paths
	list(REMOVE_ITEM CSM_LIBRARY_DIRS ".")

	MESSAGE(STATUS "CSM_LIBRARY_DIRS: ${CSM_LIBRARY_DIRS}")
	MESSAGE(STATUS "CSM_LIBRARIES: ${CSM_LIBRARIES}")
	MESSAGE(STATUS "CSM_INCLUDE_DIRS: ${CSM_INCLUDE_DIRS}")
	
	INCLUDE_DIRECTORIES(${CSM_INCLUDE_DIRS}) # important! 
	LINK_DIRECTORIES(${CSM_LIBRARY_DIRS})    # important! 
ELSE(${CSM_FOUND})	
	MESSAGE(FATAL_ERROR "CSM not found. Check that the environment \
	variable PKG_CONFIG_PATH includes the path containing the file 'csm.pc'.")
ENDIF(${CSM_FOUND})		

############################### Boost ###############################################
find_package(Boost 1.43 COMPONENTS system random
# Don't need these:   serialization  filesystem thread date_time 
    REQUIRED)
include_directories(cpp ${Boost_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})

############################### Library ###############################################

set(planar_icp_lib_source 
    cpp/planarICP.cpp
    cpp/csm_interface.cpp
    cpp/planarICP_matlab.cpp
    cpp/GslErrorHandler.cpp
    cpp/Saliency.cpp
    cpp/FilteringUtility.cpp
)
set(planar_icp_tests_source 
    tests/testPlanarICP.cpp 
    tests/testPLICP.cpp
    tests/testSaliency.cpp
    tests/testFilteringUtility.cpp
)

# Build shared library 
# Include installed versions
add_library(planar_icp-shared SHARED ${planar_icp_lib_source})
target_link_libraries(planar_icp-shared ${Boost_SYSTEM_LIBRARY_DEBUG} csm gsl gslcblas ${gtsam_deps})
set_target_properties(planar_icp-shared PROPERTIES 
    OUTPUT_NAME         planar_icp
    CLEAN_DIRECT_OUTPUT 1)
install(TARGETS planar_icp-shared EXPORT planar_icp-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
list(APPEND planar_icp_EXPORTED_TARGETS planar_icp-shared)
#set(planar_icp_EXPORTED_TARGETS "${planar_icp_EXPORTED_TARGETS}" PARENT_SCOPE)

install(TARGETS planar_icp-shared LIBRARY DESTINATION lib )

############################### Tests ###############################################
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
foreach(test_src_file ${planar_icp_tests_source})
    get_filename_component(test_base ${test_src_file} NAME_WE)
    message(STATUS "Adding test ${test_src_file} with base name ${test_base}" )
    add_executable(${test_base} ${test_src_file})
    target_link_libraries(${test_base} planar_icp-shared ${gtsam_deps} ${CSM_LIBRARIES} CppUnitLite)
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check ${test_base})
endforeach(test_src_file)

############################### Wrapper ###############################################
# Matlab wrapping (adapted from ddfsam)
if (NOT "${GtsamMatlabWrap}" STREQUAL "")
	include(GtsamMatlabWrap)

	# NOTE: compile flags need to be split out here because mex compiling doesn't use any of
	# cmake's tools for adding paths/libraries. 

	# Include flags 
	set(CSM_INCLUDE_FLAGS "")
	foreach(include_path ${CSM_INCLUDE_DIRS})
	    set(CSM_INCLUDE_FLAGS "-I${include_path} ${CSM_INCLUDE_FLAGS}")
	endforeach()
	message(STATUS "CSM_INCLUDE_FLAGS: [${CSM_INCLUDE_FLAGS}]")

	# Linker paths
	set(CSM_LIBRARY_DIR_FLAGS "")
	foreach(link_path ${CSM_LIBRARY_DIRS})
	    set(CSM_LIBRARY_DIR_FLAGS "-L${link_path} ${CSM_LIBRARY_DIR_FLAGS}")
	endforeach()
	message(STATUS "CSM_LIBRARY_DIR_FLAGS: [${CSM_LIBRARY_DIR_FLAGS}]")

	# Library flags
	set(CSM_LIBRARY_FLAGS "")
	foreach(lib ${CSM_LIBRARIES})
	    set(CSM_LIBRARY_FLAGS "-l${lib} ${CSM_LIBRARY_FLAGS}")
	endforeach()

	# Standard configuration
	set(PLANAR_ICP_TOOLBOX_FLAGS 
	#    ${GTSAM_BUILD_MEX_BINARY_FLAGS} 
	#    -I${PROJECT_SOURCE_DIR} -I${PROJECT_SOURCE_DIR}/cpp
	#    -I${Boost_INCLUDE_DIR} -I${MEX_INCLUDE_ROOT} 
	#    ${CSM_INCLUDE_FLAGS} ${CSM_LIBRARY_DIR_FLAGS} ${CSM_LIBRARY_FLAGS}
	#    -I${GTSAM_INCLUDE_DIR} -I${WRAP_HEADER_PATH} -Wl,-rpath,${CMAKE_BINARY_DIR}:${CMAKE_INSTALL_PREFIX}/lib
	    )
	set(PLANAR_ICP_LIBRARY_DEPS ${gtsam_deps} planar_icp-shared)
	set(GTSAM_BUILD_MEX_BIN ON)

	# User-friendly wrapping function.  Builds a mex module from the provided
	# interfaceHeader.  For example, for the interface header /path/to/gtsam.h,
	# this will build the wrap module 'gtsam'.
	# Params:
	#   interfaceHeader  : Absolute or relative path to the interface definition file
	#   linkLibraries    : All dependent CMake target names, library names, or full library paths
	#   extraIncludeDirs : Extra include directories, in addition to those already passed to include_directories(...)
	#   extraMexFlags    : Any additional compiler flags
	wrap_and_install_library("planar_icp.h" "${PLANAR_ICP_LIBRARY_DEPS}" "" "${PLANAR_ICP_TOOLBOX_FLAGS}")
endif()
# Install config and export files
GtsamMakeConfigFile(planar_icp)
# export(TARGETS ${planar_icp_EXPORTED_TARGETS} FILE planar_icp-exports.cmake)
