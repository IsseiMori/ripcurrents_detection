project(bbDelete)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

file(GLOB bbDelete_SRC
    "*.hpp"
    "*.cpp"
)

add_executable( bbDelete ${bbDelete_SRC})
target_compile_features(bbDelete PUBLIC cxx_lambdas)

