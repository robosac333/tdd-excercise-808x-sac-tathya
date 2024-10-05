add_library(tdd STATIC tdd.cpp)
target_include_directories(tdd PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(tdd PUBLIC ${COMMON_LIBS})
target_link_options(tdd PUBLIC --static)