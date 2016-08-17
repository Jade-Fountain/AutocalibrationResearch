###############################################################################
# Find OpenNI2
#
# This sets the following variables:
# OPENNI2_FOUND - True if OPENNI2 was found.
# OPENNI2_INCLUDE_DIRS - Directories containing the OPENNI2 include files.
# OPENNI2_LIBRARIES - Libraries needed to use OPENNI2.
# OPENNI2_DEFINITIONS - Compiler flags for OPENNI2.

find_package(PkgConfig)
if(${CMAKE_VERSION} VERSION_LESS 2.8.2)
  pkg_check_modules(PC_OPENNI2 openni2-dev)
else()
  pkg_check_modules(PC_OPENNI2 QUIET openni2-dev)
endif()

set(OPENNI2_DEFINITIONS ${PC_OPENNI2_CFLAGS_OTHER})

#using the 64bit version of OpenNi2 if generating for 64bit
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROGRAMFILES_ "$ENV{PROGRAMW6432}")
    set(OPENNI2_SUFFIX "64")
else(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROGRAMFILES_ "$ENV{PROGRAMFILES}")
    set(OPENNI2_SUFFIX "")
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

#add a hint so that it can find it without the pkg-config
find_path(OPENNI2_INCLUDE_DIR OpenNI.h
    HINTS ${PC_OPENNI2_INCLUDEDIR} ${PC_OPENNI2_INCLUDE_DIRS} /usr/include/ni2 /usr/include/openni2 /usr/local/include/ni2 /usr/local/include/openni2 
    "${PROGRAMFILES_}/OpenNI2/Include"
    PATH_SUFFIXES openni2)
#add a hint so that it can find it without the pkg-config
find_library(OPENNI2_LIBRARY 
    NAMES OpenNI264 OpenNI2
    HINTS ${PC_OPENNI2_LIBDIR} ${PC_OPENNI2_LIBRARY_DIRS} /usr/lib/ni2 /usr/local/lib/ni2 "${PROGRAMFILES_}/OpenNI2/Lib${OPENNI2_SUFFIX}")

set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})
# message("OPENNI2_INCLUDE_DIRS: " ${OPENNI2_INCLUDE_DIR})
set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})
# message("OPENNI2_LIBRARIES: " ${OPENNI2_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2 DEFAULT_MSG
    OPENNI2_LIBRARY OPENNI2_INCLUDE_DIR)
    
mark_as_advanced(OPENNI2_LIBRARY OPENNI2_INCLUDE_DIR)
if(OPENNI2_FOUND)
  include_directories(${OPENNI2_INCLUDE_DIRS})
  message(STATUS "OpenNI2 found (include: ${OPENNI2_INCLUDE_DIR}, lib: ${OPENNI2_LIBRARY})")
endif(OPENNI2_FOUND)