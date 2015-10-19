
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
TARGET_LINK_LIBRARIES(${CURRENT_FOLDER} ${COMMON_LIBRARIES})
