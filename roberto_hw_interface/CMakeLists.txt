cmake_minimum_required(VERSION 3.0.2)
project(roberto_hw_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# COPY PASTE FROM THE VANDERBILT REPO
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf")
    set(CTRE_ARCH "raspbian")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")
    set(CTRE_ARCH "aarch64bionic")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "x86_64-linux-gnu")
    set(CTRE_ARCH "x86-64")
else()
    message(FATAL_ERROR "Unsupported architecture for CTRE Phoenix: ${CMAKE_LIBRARY_ARCHITECTURE}")
endif()

set(CTRE_VERSION "5.19.4")

# Macro for downloading CTRE Libraries
macro(add_ctre_library LIB_NAME URL_NAME)
    SET(LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/ctre/${CTRE_VERSION}/${URL_NAME}")
    SET(LIB_DEST "${LIB_DIR}/linux/${CTRE_ARCH}/static/lib${LIB_NAME}.a")
    file(MAKE_DIRECTORY ${LIB_DIR})
    file(MAKE_DIRECTORY ${LIB_DIR}/ctre)

    # Download the library
    set(FILE_NAME "${URL_NAME}-${CTRE_VERSION}-linux${CTRE_ARCH}static.zip")
    set(FILE_URL "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/${URL_NAME}/${CTRE_VERSION}/${FILE_NAME}")
    set(FILE_DEST "${LIB_DIR}/${FILE_NAME}")
    if(NOT EXISTS ${FILE_DEST})
        message(STATUS "Downloading ${LIB_NAME} from ${FILE_URL}")
        file(DOWNLOAD ${FILE_URL} ${FILE_DEST})
    endif()

    # Download the headers
    SET(HEADER_NAME "${URL_NAME}-${CTRE_VERSION}-headers.zip")
    set(HEADER_URL "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/${URL_NAME}/${CTRE_VERSION}/${HEADER_NAME}")
    set(HEADER_DEST "${LIB_DIR}/${HEADER_NAME}")
    if(NOT EXISTS ${HEADER_DEST})
        message(STATUS "Downloading headers for ${LIB_NAME} from ${FILE_URL}")
        file(DOWNLOAD ${HEADER_URL} ${HEADER_DEST})
    endif()

    # Create a target for extracing the zip files
    add_custom_target(${LIB_NAME}-unzip DEPENDS ${LIB_DEST})
    add_custom_command(OUTPUT ${LIB_DEST} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${FILE_DEST}
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${HEADER_DEST}
        WORKING_DIRECTORY ${LIB_DIR}
        COMMENT "Unpacking ${LIB_NAME}"
        VERBATIM
    )

    # Add library
    add_library(${LIB_NAME} STATIC IMPORTED)
    add_dependencies(${LIB_NAME} ${LIB_NAME}-unzip)
    set_target_properties(${LIB_NAME} PROPERTIES
        IMPORTED_LOCATION ${LIB_DEST}
        INTERFACE_INCLUDE_DIRECTORIES ${LIB_DIR}
    )

    # Mark files for installation by catkin
    install (
        FILES ${LIB_DEST}
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
    install (
        DIRECTORY ${LIB_DIR}/ctre
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
endmacro()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

set(PHOENIX_LIBRARIES
    CTRE_Phoenix
    CTRE_PhoenixCCI
    CTRE_PhoenixDiagnostics
    CTRE_PhoenixPlatform_socketcan
    CTRE_PhoenixCanutils
    CTRE_PhoenixCore
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PHOENIX_LIBRARIES}
  CATKIN_DEPENDS controller_manager roscpp rospy std_msgs std_srvs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_ctre_library(CTRE_Phoenix                   api-cpp)
add_ctre_library(CTRE_PhoenixCCI                cci)
add_ctre_library(CTRE_PhoenixDiagnostics        diagnostics)
add_ctre_library(CTRE_PhoenixPlatform_socketcan platform-socketcan)
add_ctre_library(CTRE_PhoenixCanutils           canutils)
add_ctre_library(CTRE_PhoenixCore               core)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/roberto_hw_interface.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/roberto_hw_interface_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_roberto_hw_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(roberto_hw src/roberto_hw.cpp)
target_link_libraries(roberto_hw 
  ${catkin_LIBRARIES}
  Threads::Threads
  ${PHOENIX_LIBRARIES}
)

add_executable(falcon_test src/falcon_test.cpp)
target_link_libraries(falcon_test
  ${catkin_LIBRARIES}
  Threads::Threads
  ${PHOENIX_LIBRARIES}
)

add_executable(topic_convert src/topic_convert.cpp)
target_link_libraries(topic_convert ${catkin_LIBRARIES})
