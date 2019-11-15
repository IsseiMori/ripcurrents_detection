cmake_minimum_required(VERSION 3.14)
project(bbResize)
find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

file(GLOB bbResize_SRC
    "*.hpp"
    "*.cpp"
)

add_executable( bbResize ${bbResize_SRC})
target_compile_features(bbResize PUBLIC cxx_lambdas)
target_link_libraries( bbResize ${OpenCV_LIBS} )

