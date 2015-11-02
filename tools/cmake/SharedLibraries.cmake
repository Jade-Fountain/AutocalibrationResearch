# Find our globally shared libraries:
FIND_PACKAGE(NUClear REQUIRED)
FIND_PACKAGE(armadillo REQUIRED)
FIND_PACKAGE(Protobuf REQUIRED)
FIND_PACKAGE(CATCH REQUIRED)
FIND_PACKAGE(YAML-CPP REQUIRED)
FIND_PACKAGE(muParser REQUIRED)
FIND_PACKAGE(CPPFormat REQUIRED)
FIND_PACKAGE(OpenBLAS REQUIRED)
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
FIND_PACKAGE(LAPACK REQUIRED)
#PSMOVE AND GRAPHICS
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(GLFW REQUIRED)
FIND_PACKAGE(GLM REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)
FIND_PACKAGE(JPEG REQUIRED)
FIND_PACKAGE(PNG REQUIRED)
FIND_PACKAGE(png++ REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system filesystem REQUIRED)
FIND_PACKAGE(assimp REQUIRED)
FIND_PACKAGE(OPENCV REQUIRED)
FIND_PACKAGE(LIBUSB REQUIRED)
FIND_PACKAGE(SFML 2 COMPONENTS system window graphics audio REQUIRED)

find_library(FOUNDATION Foundation)
find_library(AVFOUNDATION AVFoundation)
find_library(IOKIT IOKit)
find_library(COREFOUNDATION CoreFoundation)
find_library(IOBLUETOOTH IOBluetooth)
list(APPEND PSMOVEAPI_REQUIRED_LIBS ${FOUNDATION})
list(APPEND PSMOVEAPI_REQUIRED_LIBS ${AVFOUNDATION})
list(APPEND PSMOVEAPI_REQUIRED_LIBS ${IOKIT})
list(APPEND PSMOVEAPI_REQUIRED_LIBS ${COREFOUNDATION})
list(APPEND PSMOVEAPI_REQUIRED_LIBS ${IOBLUETOOTH})


# Set include directories and libraries:
INCLUDE_DIRECTORIES(SYSTEM ${NUClear_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${BLAS_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${PROTOBUF_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${CATCH_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${YAML-CPP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${muParser_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${CPPFormat_INCLUDE_DIRS})

#PSMOVE AND GRAPHICS

#psmove
set(PSMOVEAPI_INCLUDE_DIR /usr/local/psmoveapi/include)
set(PSMOVEAPI_LIBRARY #/usr/local/psmoveapi/lib64/libpsmoveapi.dylib
                      /usr/local/psmoveapi/lib64/libpsmoveapi_static.a
                      #/usr/local/psmoveapi/lib64/libpsmoveapi.3.1.0.dylib
    ) 
set(PSMOVEAPI_TRACKER_LIBRARY #/usr/local/psmoveapi/lib64/libpsmoveapi_tracker.dylib
                              /usr/local/psmoveapi/lib64/libpsmoveapi_tracker_static.a
                              #/usr/local/psmoveapi/lib64/libpsmoveapi_tracker.3.1.0.dylib
                              )

#soil image lib
set(SOIL_INCLUDE_DIR /usr/local/include)
set(SOIL_LIBRARY /usr/local/lib/libSOIL.a)


INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${SFML_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${GLFW_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${GLM_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${PNG_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(SYSTEM ${libpng++_INCLUDE_DIRS} REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${SOIL_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${LibUSB_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${ASSIMP_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${PSMOVEAPI_INCLUDE_DIR} REQUIRED)
INCLUDE_DIRECTORIES(${PSMOVEAPI_TRACKER_INCLUDE_DIR} REQUIRED)

    
SET(NUBOTS_SHARED_LIBRARIES
    ${NUClear_LIBRARIES}
    ${BLAS_LIBRARIES}
    ${LIBGFORTRAN_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${YAML-CPP_LIBRARIES}
    ${muParser_LIBRARIES}
    ${CPPFormat_LIBRARIES}
    ${OPENGL_gl_LIBRARY}
    ${GLEW_LIBRARY}
    ${GLFW_LIBRARIES}
    ${SFML_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${PNG_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${ASSIMP_LIBRARIES}
    ${SOIL_LIBRARY}
    ${LibUSB_LIBRARIES}
    ${OpenCV_LIBS}
    ${PSMOVEAPI_LIBRARY}
    ${PSMOVEAPI_TRACKER_LIBRARY}
    ${ARMADILLO_LIBRARY}
    ${PSMOVEAPI_REQUIRED_LIBS}
)
