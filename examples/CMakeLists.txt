include_directories (
    ${libseek-thermal_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${LIBUSB_INCLUDE_DIRS}

)

link_libraries (
    seek_static
    ${OpenCV_LIBRARIES}
    ${LIBUSB_LIBRARIES}
)

add_executable (seek_test seek_test.cpp)
add_executable (seek_test_pro seek_test_pro.cpp)
add_executable (seek_viewer seek_viewer.cpp args.h)
add_executable (seek_create_flat_field seek_create_flat_field.cpp)
add_executable (seek_snapshot seek_snapshot.cpp)
add_executable (seek_viewer_fb seek_viewer_fb.cpp args.h)

install (TARGETS
    seek_test
    seek_test_pro
    seek_viewer
    seek_create_flat_field
    seek_snapshot
    seek_create_flat_field
    DESTINATION "bin"
)
