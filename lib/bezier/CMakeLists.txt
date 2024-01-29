file(GLOB SRC_FILES src/*.cpp)
add_library(bezier ${SRC_FILES})
target_include_directories(bezier PUBLIC include)
