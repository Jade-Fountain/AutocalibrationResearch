
MESSAGE("Adding subdirectory " ${CURRENT_FOLDER} )


INCLUDE_DIRECTORIES("." ".." "../common")

FILE(GLOB_RECURSE src_files
      "**.cpp"
    )

set(src_files ${src_files})

MESSAGE("source files " src_files)
# Add Executables
SET(fusion_src ${inc_files} ${src_files})

# Executable named after subdir
ADD_EXECUTABLE (${CURRENT_FOLDER} ${fusion_src})
SET_TARGET_PROPERTIES(${CURRENT_FOLDER} PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(${CURRENT_FOLDER} ${LIBRARIES})
message( CMAKE_BINARY_DIR ${CMAKE_BINARY_DIR})
SET_PROPERTY(TARGET ${CURRENT_FOLDER} PROPERTY 
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
