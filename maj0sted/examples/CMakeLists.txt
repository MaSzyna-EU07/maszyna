add_executable(maj0sted_example basic_editor.cpp)
target_link_libraries(maj0sted_example PRIVATE maj0sted::maj0sted)

add_executable(maj0sted_plan_to_svg plan_to_svg.cpp)
target_link_libraries(maj0sted_plan_to_svg PRIVATE maj0sted::maj0sted)
