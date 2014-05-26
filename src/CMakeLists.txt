FILE (GLOB_RECURSE project_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
SET (project_LIBS ${REIKI_LIBS})
SET (project_BIN ${PROJECT_NAME})

IF (NOT DEFINED project_build_static)
   SET(project_build_static 0)
ENDIF (NOT DEFINED project_build_static)

IF (NOT DEFINED project_build_shared)
   SET(project_build_shared 1)
ENDIF (NOT DEFINED project_build_shared)

IF (project_build_static)
	ADD_LIBRARY(staticlib STATIC ${project_SRCS})
	TARGET_LINK_LIBRARIES(staticlib ${project_LIBS})
	SET_TARGET_PROPERTIES(staticlib PROPERTIES VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}" OUTPUT_NAME ${project_BIN} CLEAN_DIRECT_OUTPUT 1)
	INSTALL(TARGETS staticlib DESTINATION lib)
ENDIF()

IF (project_build_shared)
	ADD_LIBRARY(sharedlib SHARED ${project_SRCS})
	SET_TARGET_PROPERTIES(sharedlib PROPERTIES VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}" OUTPUT_NAME ${project_BIN} CLEAN_DIRECT_OUTPUT 1)
	TARGET_LINK_LIBRARIES(sharedlib ${project_LIBS})
	INSTALL(TARGETS sharedlib DESTINATION lib)
ENDIF()

###### CppLint #######
add_custom_command(
    TARGET staticlib PRE_BUILD
    COMMAND find ${CMAKE_SOURCE_DIR}/src/reiki ${CMAKE_SOURCE_DIR}/include/reiki -not -path '*.in.cmake' | xargs python ${CMAKE_SOURCE_DIR}/cpplint.py --filter=-build/header_guard,-whitespace/line_length,-whitespace/labels,-legal/copyright,-readability/todo,-build/include,-readability/streams,-readability/function,-runtime/arrays,-whitespace/comments,-whitespace/semicolon,-build/namespaces,-runtime/int,-runtime/references,-build/printf_format
    COMMENT "Style checking"
)
