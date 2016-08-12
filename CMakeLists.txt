CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(Autocalibration)

SET(VERSION_MAJOR 0)
SET(VERSION_MINOR 2)
SET(VERSION_PATCH 0)

SET(NUCLEAR_ROLE_BANNER_FILE "${CMAKE_SOURCE_DIR}/tools/cmake/banner.txt" CACHE PATH "The path the banner to print at the start of each role execution" FORCE)
# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/tools/cmake/")

# Setup our compiler settings
INCLUDE(Compiler)

# Setup our shared includes
INCLUDE(SharedLibraries)

#Use nuclear roles
ADD_SUBDIRECTORY(nuclear)