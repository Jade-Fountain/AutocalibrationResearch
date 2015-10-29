# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

# Custom target to copy and install configuration files and binarys to the robot
ADD_CUSTOM_TARGET(send
                  COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/tools/scripts/send.py" "--robot_ip=\${robot}" "--config=\${config}" "--username=\${username}"
                  DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/scripts/send.py")

# Put it in an IDE group for shared
SET_PROPERTY(TARGET send PROPERTY FOLDER "ALL_BUILD/")

# Ninja code!
FOREACH(robot 0 1 2 3 4 5 6 7)
    FOREACH(config "" u o n i)
        FOREACH(ethernet "" e)
            IF("${robot}" STREQUAL "0")
                SET(address "10.1.0.1")
            ELSEIF("${ethernet}" STREQUAL "e")
                SET(address "10.1.2.${robot}")
            ELSE()
                SET(address "10.1.1.${robot}")
            ENDIF()

            # Make our installer
            ADD_CUSTOM_TARGET("d${robot}${ethernet}${config}"
                COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/tools/scripts/send.py" "--robot_ip=${address}" "--config=${config}" "--username=darwin"
                DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/scripts/send.py")


            # Move our installer to an IDE group
            SET_PROPERTY(TARGET "d${robot}${ethernet}${config}" PROPERTY FOLDER "installers")

        ENDFOREACH(ethernet)
    ENDFOREACH(config)
ENDFOREACH(robot)

ADD_CUSTOM_TARGET("dall"
          DEPENDS d1eo d2eo d3eo d4eo d5eo d6eo)

ADD_CUSTOM_TARGET("nc"
          COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/tools/scripts/send.py" "--robot_ip=10.0.1.120" "--username=nubots"
          DEPENDS ${NUCLEAR_ROLES} "${CMAKE_SOURCE_DIR}/tools/scripts/send.py")
