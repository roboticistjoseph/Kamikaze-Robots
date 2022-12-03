add_executable(kamikaze_app main.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(kamikaze_app PUBLIC kamikaze_library ${OpenCV_LIBS})
