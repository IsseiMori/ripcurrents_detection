cmake_minimum_required(VERSION 3.14)
project(bbAdder)
find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

file(GLOB bbAdder_SRC
    "*.hpp"
    "*.cpp"
)

add_executable( bbAdder ${bbAdder_SRC})
target_compile_features(bbAdder PUBLIC cxx_lambdas)
target_link_libraries( bbAdder ${OpenCV_LIBS} )

