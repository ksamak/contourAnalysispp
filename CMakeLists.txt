# <Auboyneau Vincent> <GNU General Public License version 3 (GPLv3)>.

#
# Project Properties
#
CMAKE_MINIMUM_REQUIRED (VERSION 2.6.2)
PROJECT (contourAnalysispp C CXX)
include( FindPkgConfig )
SET (APPLICATION_NAME "contourAnalysispp")
SET (APPLICATION_CODENAME "capp")
SET (APPLICATION_COPYRIGHT_YEARS "2014")
SET (APPLICATION_VERSION_MAJOR 0)
SET (APPLICATION_VERSION_MINOR 0)
SET (APPLICATION_VERSION_PATCH 1)
SET (APPLICATION_VERSION_TYPE SNAPSHOT)
SET (APPLICATION_VERSION_STRING "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}.${APPLICATION_VERSION_PATCH}-${APPLICATION_VERSION_TYPE}")
SET (APPLICATION_ID "${PROJECT_NAME}")

#Compute exact revision from git if possible
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmakemodules/")
INCLUDE(getgitrevisiondescription)
# run git describe "--tags"
GIT_DESCRIBE(GIT_VERSION_STRING "--tags" "--always")
#if not possible use version number we ship instead
IF(NOT VERSION_STRING)
    set(VERSION_STRING APPLICATION_VERSION_STRING)
ENDIF(NOT VERSION_STRING)

#
# Debugging Options
#
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O3" )
SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -W -std=c++0x" )

#if("${REIKI_BUILD_WITH_TRACES}" STREQUAL "")
#  set(REIKI_BUILD_WITH_TRACES "1")
#endif()

#
# Project Output Paths
#
SET (MAINFOLDER ${PROJECT_SOURCE_DIR})
SET (EXECUTABLE_OUTPUT_PATH "${MAINFOLDER}/bin")
SET (LIBRARY_OUTPUT_PATH "${MAINFOLDER}/lib")
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})


#
# Project Search Paths
#
#LIST (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}")
#LIST (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}/tools")
#LIST (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}/thirdparty")
#LIST (APPEND CMAKE_MODULE_PATH "${MAINFOLDER}/tools/share/cmake")
INCLUDE_DIRECTORIES("${MAINFOLDER}/include")
INCLUDE_DIRECTORIES("${MAINFOLDER}/src")


#
# Locate Project Prerequisites
#
pkg_search_module(OPENCV REQUIRED opencv opencv2)
SET(OPENCV_CAPP_LIBS opencv_core opencv_imgproc opencv_ml)

foreach(f ${OPENCV_LIBRARY_DIRS})
    message(STATUS "Using opencv libs at " ${f})
endforeach(f)

INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OPENCV_LIBRARY_DIRS})


##Provide sane default values for flags
#IF ("${BUILD_TEST}" STREQUAL "")
#  SET (BUILD_TEST "0")
#ENDIF()

IF ("${BUILD_EXAMPLES}" STREQUAL "")
    SET (BUILD_EXAMPLES "0")
ENDIF()

#
# Add Build Targets
#
ADD_SUBDIRECTORY(src)
#IF ( ${BUILD_TEST} )
#    ADD_SUBDIRECTORY(test)
#ENDIF()

IF ( ${BUILD_EXAMPLES} )
    ADD_SUBDIRECTORY(examples)
ENDIF()

#
# Add Install Targets
#
IF (EXISTS "${MAINFOLDER}/include/${PROJECT_NAME}" AND IS_DIRECTORY "${MAINFOLDER}/include/${PROJECT_NAME}")
    INSTALL(DIRECTORY "${MAINFOLDER}/include/${PROJECT_NAME}" DESTINATION "include")
ENDIF (EXISTS "${MAINFOLDER}/include/${PROJECT_NAME}" AND IS_DIRECTORY "${MAINFOLDER}/include/${PROJECT_NAME}")

install(DIRECTORY src/ressources
    DESTINATION share/contourAnalysispp
)

#Install pkg-config
FOREACH (CONFIGINPUT ${CONFIGINPUTS5})
    STRING (REPLACE ".pc.in.cmake" ".pc" CONFIGOUTPUT "${CONFIGINPUT}")
    INSTALL(FILES "${CONFIGOUTPUT}" DESTINATION lib/pkgconfig)
ENDFOREACH (CONFIGINPUT ${CONFIGINPUTS5})

#
# Add Documentation Targets
#
#INCLUDE (DocumentationTargets)

###### CppLint #######
add_custom_command(
    TARGET axtdocprocess PRE_BUILD
    COMMAND find ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include | xargs python ${CMAKE_SOURCE_DIR}/cpplint.py
    --filter=-build/header_guard,-whitespace/line_length,-whitespace/labels,-legal/copyright,-readability/todo,-build/include,-readability/streams,-readability/function,-runtime/arrays,-whitespace/comments,-whitespace/semicolon,-build/namespaces,-runtime/int,-runtime/references,-build/printf_format | echo
    COMMENT "Style checking"
    )

