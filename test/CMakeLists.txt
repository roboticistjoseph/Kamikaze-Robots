set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp
    test.cpp
)

link_directories (${CMAKE_BINARY_DIR}/src)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC kamikaze_library gtest ${OpenCV_LIBS})