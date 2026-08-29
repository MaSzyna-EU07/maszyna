add_executable(maj0sted_test_chain test_chain.cpp)
target_link_libraries(maj0sted_test_chain PRIVATE maj0sted::maj0sted)
target_include_directories(maj0sted_test_chain PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME chain COMMAND maj0sted_test_chain)

add_executable(maj0sted_test_hold test_hold.cpp)
target_link_libraries(maj0sted_test_hold PRIVATE maj0sted::maj0sted)
target_include_directories(maj0sted_test_hold PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME hold COMMAND maj0sted_test_hold)

add_executable(maj0sted_test_lines test_lines.cpp)
target_link_libraries(maj0sted_test_lines PRIVATE maj0sted::maj0sted)
target_include_directories(maj0sted_test_lines PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME lines COMMAND maj0sted_test_lines)

add_executable(maj0sted_test_graph test_graph.cpp)
target_link_libraries(maj0sted_test_graph PRIVATE maj0sted::maj0sted)
target_include_directories(maj0sted_test_graph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME graph COMMAND maj0sted_test_graph)

add_executable(maj0sted_test_format test_format.cpp)
target_link_libraries(maj0sted_test_format PRIVATE maj0sted::maj0sted)
target_include_directories(maj0sted_test_format PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME format COMMAND maj0sted_test_format)

add_executable(maj0sted_test_turnout test_turnout.cpp)
target_link_libraries(maj0sted_test_turnout PRIVATE maj0sted::maj0sted)
target_include_directories(maj0sted_test_turnout PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME turnout COMMAND maj0sted_test_turnout)

add_executable(maj0sted_test_tile_cache test_tile_cache.cpp)
target_link_libraries(maj0sted_test_tile_cache PRIVATE maj0sted::maj0sted)
target_include_directories(maj0sted_test_tile_cache PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME tile_cache COMMAND maj0sted_test_tile_cache)

