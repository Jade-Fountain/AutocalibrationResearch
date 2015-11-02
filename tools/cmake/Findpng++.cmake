################################################################################
#                                                                              #
#                              Louis-Kenzo Cahier                              #
#                                                                              #
################################################################################

#Find the path to the headers
find_path(libpng++_INCLUDE_DIRS png.hpp
                                /usr/include/png++
                                /usr/local/include/png++)

#Set the Module Finder output variables
if (libpng++_INCLUDE_DIRS)
	# the png++ directory doesn't contain any file so we can't find_path it, but it's the real root of the png++ installation
	set(libpng++_INCLUDE_DIRS ${libpng++_INCLUDE_DIRS}/..)
	set(libpng++_FOUND TRUE)
endif()
set(libpng++_INCLUDE_DIR ${libpng++_INCLUDE_DIRS})

if (libpng++_FOUND)
	if (NOT libpng++_FIND_QUIETLY)
		message(STATUS "Found libpng++: ${libpng++_INCLUDE_DIRS}")
	endif()
else (libpng++_FOUND)
	if (libpng++_FIND_REQUIRED)
		message(FATAL_ERROR "Could not find libpng++")
	endif()
endif()

################################################################################
