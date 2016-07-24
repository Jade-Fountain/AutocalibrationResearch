#GetNUtilities.cmake
MESSAGE("** Configuring NUtilities...")

SET(NUTILITIES_SRC_FILES "")

#Get all subdirectories recursively
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  #Check for folders
  FOREACH(child ${children})
	IF(IS_DIRECTORY ${curdir}/${child})
		message("Dir ${child} : ${curdir}/${child}")
		LIST(APPEND dirlist ${child})
	ENDIF()
  ENDFOREACH()

  message("dirlist = ${dirlist}")
  #Do subdirectories
  SET(subdirlist "")
  FOREACH(child ${dirlist})
  	SUBDIRLIST(sub_folders "${curdir}/${child}")
  	LIST(APPEND subdirlist ${sub_folders})
  	message("appending ${subdirlist} with ${sub_folders}. current folder ${child}")
  ENDFOREACH()

  message("subdirlist ${subdirlist}")
  message("dirlist ${dirlist}")
  SET(result ${dirlist} ${subdirlist})
  message("result ${result}")
ENDMACRO()

#Check if a list contains an entry ${value}
MACRO(LIST_CONTAINS var value)
  SET(${var})
  FOREACH (value2 ${ARGN})
    IF (${value} STREQUAL ${value2})
      SET(${var} TRUE)
    ENDIF (${value} STREQUAL ${value2})
  ENDFOREACH (value2)
ENDMACRO(LIST_CONTAINS)

#Collect CXX files
MACRO(GET_CXX_FILES file_list dir)
	FILE(GLOB file_list 
				"${dir}*.cpp" 
				"${dir}*.hpp" 
				"${dir}*.h" 
				"${dir}*.c" 
				"${dir}/*.cpp" 
				"${dir}/*.hpp"
				"${dir}/*.c"
				"${dir}/*.h")
ENDMACRO(GET_CXX_FILES)
#Collect CXX files recursively
MACRO(GET_CXX_FILES_RECURSE file_list dir)
	FILE(GLOB_RECURSE file_list 
				"${dir}*.cpp" 
				"${dir}*.hpp" 
				"${dir}*.h" 
				"${dir}*.c" 
				"${dir}/*.cpp" 
				"${dir}/*.hpp"
				"${dir}/*.c"
				"${dir}/*.h")
ENDMACRO(GET_CXX_FILES_RECURSE)

# Get source files based on configuration of libraries and project
IF(${PROJECT_NAME} STREQUAL NUbots)
	MESSAGE("** Using NUbots utilities...")
	#Set folder specification style
	SET(EXCLUDE_FOLDERS TRUE)
	#Specify folders
	SET(FOLDERS
		autocal
		)

ELSEIF(${PROJECT_NAME} STREQUAL Autocalibration)
	MESSAGE("** Using Autocalibration utilities...")
	#Set folder specification style
	SET(EXCLUDE_FOLDERS TRUE)
	#Specify folders
	SET(FOLDERS
		autocal
		error
		file
		kdtree
		math/matrix
		math/angle
		math/comparison
		math/coordinates
		math/ransac
		math/optimisation
		math/filter
		math/geometry
		strutil
		)

ELSE()
	message( FATAL_ERROR "NO UTILITIES LOADED - please check conditions for loading utilities" )
ENDIF()

#Specify by inclusion
IF(INCLUDE_FOLDERS)
	FOREACH(folder ${FOLDERS})
		GET_CXX_FILES_RECURSE(src "${NUTILITIES_DIR}/${folder}")
		SET(NUTILITIES_SRC_FILES ${NUTILITIES_SRC_FILES} ${src})
	ENDFOREACH()

#Specify by exclusion
ELSEIF(EXCLUDE_FOLDERS)
	SUBDIRLIST(NUTILITIES_FOLDERS ${NUTILITIES_DIR})
	FOREACH(dir ${NUTILITIES_FOLDERS})
		MESSAGE("FINAL DIR : " ${dir})
	ENDFOREACH()
	message(NUTILITIES_FOLDERS ${NUTILITIES_FOLDERS})
	FOREACH(folder ${NUTILITIES_FOLDERS})
		#Check each folder not excluded
		LIST_CONTAINS(contains ${folder} ${FOLDERS})
		IF(NOT contains)
			MESSAGE("Folder included: ${folder}")
			GET_CXX_FILES(src "${NUTILITIES_DIR}/${folder}")
			SET(NUTILITIES_SRC_FILES ${NUTILITIES_SRC_FILES} ${src})
		ENDIF()
	ENDFOREACH()

#Error
ELSE()
	message( FATAL_ERROR "FOLDER SPECIFICATION NOT SUPPORTED - Set either INCLUDE_FOLDERS or EXCLUDE_FOLDERS to TRUE" )
ENDIF()

# MESSAGE(NUTILITIES_SRC_FILES ${NUTILITIES_SRC_FILES})

