 include_directories(${PROJECT_SOURCE_DIR}/include)

 

add_executable(facedetect main.cpp)
target_link_libraries(facedetect 
    face_lib 
    openvino::runtime
    ${InferenceEngine_LIBRARIES}
    ${catkin_LIBRARIES}
    )
