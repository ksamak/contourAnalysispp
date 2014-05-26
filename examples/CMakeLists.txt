
add_executable(
    recogPattern
    recogPattern.cpp
    )

target_link_libraries (
    recogPattern
    contourAnalysispp
    ${OPENCV_LIBRARIES}
    )

install(
    TARGETS recogPattern
    DESTINATION bin
    )

