set (HEADERS
    SeekCam.h
    SeekDevice.h
    seek.h
    SeekLogging.h
    SeekThermal.h
    SeekThermalPro.h
    framebuffer.h
)

set (SOURCES
    SeekCam.cpp
    SeekDevice.cpp
    SeekThermal.cpp
    SeekThermalPro.cpp
    framebuffer.cpp
)

set (SRC ${SOURCES} ${HEADERS})

include_directories (
    ${LIBUSB_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_library (seek_static STATIC ${SRC})
add_library (seek SHARED ${SRC})

target_link_libraries (seek
    ${OpenCV_LIBS}
    ${LIBUSB_LIBRARY}
)

# install library target and header files
install (TARGETS
    seek
    DESTINATION "lib"
)

install (FILES ${HEADERS}
    DESTINATION "include/seek"
)
