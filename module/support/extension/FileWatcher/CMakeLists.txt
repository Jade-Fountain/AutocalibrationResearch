# Use libfswatch to abstract between platforms
FIND_PACKAGE(FSWatch REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${FSWatch_INCLUDE_DIRS}
               LIBRARIES ${FSWatch_LIBRARIES})
