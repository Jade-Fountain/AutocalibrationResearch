#GetNUtilities.cmake
MESSAGE("** Configuring NUtilities...")

SET(NUTILITIES_SRC_FILES "")

#Experiment
MACRO(TEST_MACRO var one two three)
	message(${one})
	message(${${two}})
	message(${three})
	SET(var ${one} ${two})
ENDMACRO(TEST_MACRO)
SET(THREE_BS b b b)
TEST_MACRO(test_result a THREE_BS c)

#Check if a list contains an entry ${value}
MACRO(LIST_CONTAINS var value)
  SET(${var})
  FOREACH (value2 ${ARGN})
    IF (${value} STREQUAL ${value2})
      SET(${var} TRUE)
    ENDIF (${value} STREQUAL ${value2})
  ENDFOREACH (value2)
ENDMACRO(LIST_CONTAINS)

#Get all subdirectories recursively
MACRO(SELECTIVE_EXPAND result curdir_ref exclude_ref)
  #Unpack args into new variables
  SET(curdir ${${curdir_ref}})
  SET(exclude ${${exclude_ref}})

  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  
  #Expand folders
  FOREACH(child ${children})
  	#Check item is a directory
	IF(IS_DIRECTORY ${curdir}/${child})
		#Check folder is not excluded
		MESSAGE("TODO: FIX THIS LINE - it doesnt work")
		LIST_CONTAINS(contains ${child} ${exclude})
		IF(NOT ${contains})
			#If not excluded, add to list of valid directories
			message("Dir ${child} : ${curdir}/${child}")
			LIST(APPEND dirlist ${child})
		ENDIF()
	ENDIF()
  ENDFOREACH()

  message("dirlist = ${dirlist}")
  
  #Expand subdirectories
  SET(subdirlist "")
  FOREACH(child ${dirlist})
  	SET(NEXT_DIR_REF "${curdir}/${child}")
  	SELECTIVE_EXPAND(sub_folders NEXT_DIR_REF ${exclude_ref})
  	LIST(APPEND subdirlist ${sub_folders})
  	message("appending ${subdirlist} with ${sub_folders}. current folder ${child}")
  ENDFOREACH()

  message("subdirlist ${subdirlist}")
  message("dirlist ${dirlist}")
  SET(result ${dirlist} ${subdirlist})
  message("result ${result}")
ENDMACRO()


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
	#Construct Exclude folders
	SET(EXCLUDE_FOLDERS "")
	FOREACH(f ${FOLDERS})
		LIST(APPEND EXCLUDE_FOLDERS "${NUTILITIES_DIR}/${f}")
	ENDFOREACH()
	#Selectively expand directory creating a list of folders containing desired
	SELECTIVE_EXPAND(NUTILITIES_FOLDERS NUTILITIES_DIR EXCLUDE_FOLDERS)
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

