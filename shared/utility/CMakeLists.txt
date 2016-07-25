CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/ ${CMAKE_SOURCE_DIR}/)

#Note where we are
SET(NUTILITIES_DIR ${CMAKE_CURRENT_SOURCE_DIR})

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

#Collect CXX files
MACRO(GET_CXX_FILES file_list dir)
	FILE(GLOB file_list 
				"${dir}*.cpp" 
				"${dir}*.c" 
				"${dir}/*.cpp" 
				"${dir}/*.c")
ENDMACRO(GET_CXX_FILES)
#Collect CXX files recursively
MACRO(GET_CXX_FILES_RECURSE file_list dir)
	FILE(GLOB_RECURSE file_list 
				"${dir}*.cpp" 
				"${dir}*.c" 
				"${dir}/*.cpp" 
				"${dir}/*.c")
ENDMACRO(GET_CXX_FILES_RECURSE)

MACRO(CHECK_DEPENDENCIES missing)
	UNSET(${missing})
	SET(dependencies ${ARGN})
	FOREACH(dep ${dependencies})
		STRING(TOUPPER ${dep} updep)
		IF(NOT ((${dep}_FOUND) OR (${updep}_FOUND)))
			LIST(APPEND ${missing} ${dep})
		ENDIF()
	ENDFOREACH()
ENDMACRO(CHECK_DEPENDENCIES)

SUBDIRLIST(directories ${CMAKE_CURRENT_SOURCE_DIR})

UNSET(NUTILITIES_LIBRARIES CACHE)
FOREACH(dir ${directories})
	ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/${dir})
ENDFOREACH()
MESSAGE("NUTILITIES LIBRARIES ${NUTILITIES_LIBRARIES}")
FOREACH(lib ${NUTILITIES_LIBRARIES})
	TARGET_LINK_LIBRARIES(${lib} ${NUTILITIES_LIBRARIES})
ENDFOREACH()

SET(NUCLEAR_UTILITY_LIBRARIES ${NUCLEAR_UTILITY_LIBRARIES} ${NUTILITIES_LIBRARIES} CACHE INTERNAL "Library for NUClear utilities" FORCE)
