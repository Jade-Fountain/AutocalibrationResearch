##cmake_minimum_required(VERSION 2.6)
##project(NUGRAPICS)
##
##SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
##
##SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O2 -Wpedantic -Wextra -Wall -Wno-unused-parameter -std=c++11" )
##
###There are lots of scripts with cmake
###for finding external libraries. 
###see /usr/local/share/cmake-2.6/Modules/Find*.cmake for more examples
##find_package(GLUT)
##find_package(OpenGL)
##find_package(GLM)
##find_package(GLFW)
##find_package(GLM)
##find_package(assimp)
##
##set(SOIL_INCLUDE_DIR /usr/local/include)
##set(SOIL_LIBRARY /usr/local/lib/libSOIL.a)
##
##INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} REQUIRED)
##INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR} REQUIRED)
##INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIR} REQUIRED)
##INCLUDE_DIRECTORIES(${GLFW_INCLUDE_DIR} REQUIRED)
##INCLUDE_DIRECTORIES(${GLM_INCLUDE_DIR} REQUIRED)
##INCLUDE_DIRECTORIES(${ASSIMP_INCLUDE_DIR} REQUIRED)
##INCLUDE_DIRECTORIES(${SOIL_INCLUDE_DIR} REQUIRED)
##
### TODO copy over shader folders
##
### Get our configuration files
##
##add_subdirectory(src)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(NUGRAPHICS_project)
SET(VERSION_MAJOR 0)
SET(VERSION_MINOR 1)
SET(VERSION_PATCH 0)

# Tell CMake where to find our CMake modules
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

set(CMAKE_MACOSX_RPATH 1)

# Compiler settings
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O0 -Wpedantic -Wextra -Wall -Wno-unused-parameter -std=c++11" )
ADD_DEFINITIONS(-DGLEW_STATIC)
ADD_DEFINITIONS(-DGLM_FORCE_RADIANS)
ADD_DEFINITIONS(-D_USE_MATH_DEFINES)

# Shared libraries
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(GLFW REQUIRED)
FIND_PACKAGE(GLM REQUIRED)
FIND_PACKAGE(JPEG REQUIRED)
FIND_PACKAGE(PNG REQUIRED)
FIND_PACKAGE(png++ REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system filesystem REQUIRED)
FIND_PACKAGE(assimp REQUIRED)
FIND_PACKAGE(armadillo REQUIRED)

#psmove
set(PSMOVEAPI_INCLUDE_DIR /usr/local/psmoveapi/include)
set(PSMOVEAPI_LIBRARY #/usr/local/psmoveapi/lib64/libpsmoveapi.dylib
                      /usr/local/psmoveapi/lib64/libpsmoveapi.3.dylib
                      #/usr/local/psmoveapi/lib64/libpsmoveapi.3.1.0.dylib
    ) 
set(PSMOVEAPI_TRACKER_LIBRARY #/usr/local/psmoveapi/lib64/libpsmoveapi_tracker.dylib
                              /usr/local/psmoveapi/lib64/libpsmoveapi_tracker.3.dylib
                              #/usr/local/psmoveapi/lib64/libpsmoveapi_tracker.3.1.0.dylib
                              )

#soil image lib
set(SOIL_INCLUDE_DIR /usr/local/include)
set(SOIL_LIBRARY /usr/local/lib/libSOIL.a)

INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${GLFW_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${GLM_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${PNG_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(SYSTEM ${libpng++_INCLUDE_DIRS} REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${SOIL_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${ASSIMP_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${PSMOVEAPI_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${ARMADILLO_INCLUDE_DIR} REQUIRED)

# ADD_SUBDIRECTORY(src/NUGL)

# Include directories
INCLUDE_DIRECTORIES("src/" ".")

FILE(GLOB_RECURSE src_files
      "src/**.cpp"
    )

set(src_files ${src_files} 
              utility/math/matrix/Transform3D.cpp
              utility/math/matrix/Rotation3D.cpp
              utility/math/geometry/UnitQuaternion.cpp
              )

# Add Executables
SET(fusion_src ${inc_files} ${src_files})
ADD_EXECUTABLE (fusion ${fusion_src})
SET_TARGET_PROPERTIES(fusion PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(fusion
    ${OPENGL_gl_LIBRARY}
    ${GLEW_LIBRARY}
    /usr/local/lib/libglfw3.3.dylib
    #${GLFW_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${PNG_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${ASSIMP_LIBRARIES}
    ${SOIL_LIBRARY}
    ${PSMOVEAPI_LIBRARY}
    ${PSMOVEAPI_TRACKER_LIBRARY}
    ${ARMADILLO_LIBRARY}
    )

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

