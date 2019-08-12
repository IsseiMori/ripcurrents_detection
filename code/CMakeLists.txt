cmake_minimum_required(VERSION 3.14)
project(RipCurrents)
find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

file(GLOB ripcurrents_SRC
    "*.hpp"
    "*.cpp"
)

add_executable( ripcurrents ${ripcurrents_SRC})
target_compile_features(ripcurrents PUBLIC cxx_lambdas)
target_link_libraries( ripcurrents ${OpenCV_LIBS} )

