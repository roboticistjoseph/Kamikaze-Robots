include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

add_library(kamikaze_library
            planner.cpp
            navigator.cpp
            ${CMAKE_SOURCE_DIR}/include/planner.hpp
            ${CMAKE_SOURCE_DIR}/include/navigator.hpp
            )

target_include_directories(kamikaze_library PUBLIC ../include)