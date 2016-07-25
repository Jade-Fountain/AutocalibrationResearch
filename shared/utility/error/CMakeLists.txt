SET(dependencies 
	ARMADILLO
	)

#Get missing dependencies
CHECK_DEPENDENCIES(missing_dependencies ${dependencies})

#Generate library name based on path
FILE(RELATIVE_PATH current_utility ${NUTILITIES_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
SET(current_library nutilities-${current_utility})

#Get file list: if we have no cxx files we dont need to link	
IF(NOT missing_dependencies)
	MESSAGE("Building Utility Library ${current_library}")
	GET_CXX_FILES_RECURSE(file_list ${CMAKE_CURRENT_SOURCE_DIR})
	IF(file_list)
		ADD_LIBRARY(${current_library} ${file_list})
		SET_TARGET_PROPERTIES(${current_library} PROPERTIES LINKER_LANGUAGE CXX)
	ELSE()
		# MESSAGE("")
	ENDIF()
ELSE()
	MESSAGE("NOT Building ${current_library}. Missing dependencies: " ${missing_dependencies})
ENDIF()

SET(NUTILITIES_LIBRARIES ${NUTILITIES_LIBRARIES} ${current_library} CACHE INTERNAL "Libraries for NUtilities" FORCE)
