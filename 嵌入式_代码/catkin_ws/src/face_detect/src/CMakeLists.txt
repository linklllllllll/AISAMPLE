 
# gflags : gflags_DIR
find_package(gflags REQUIRED)
message(STATUS "gflags status: ${GFLAGS_INCLUDE_DIR}")
include_directories(GFLAGS_INCLUDE_DIR)


# ==============

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

aux_source_directory(.  src_code)

add_library(face_lib STATIC ${src_code})

target_link_libraries(face_lib
    gflags
    utils
    monitors
    ${OpenCV_LIBS}
    openvino::runtime
    ${InferenceEngine_LIBRARIES}
    )
