cmake_minimum_required(VERSION 3.5.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components collision-bullet utils-urdf gui)
set(required_libraries dart dart-collision-bullet dart-utils-urdf dart-gui)

#if(DART_IN_SOURCE_BUILD)
#  dart_build_tutorial_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
#  return()
#endif()

find_package(DART 6.6.0 REQUIRED COMPONENTS ${required_components} CONFIG)

set(AniTk /home/alinen/projects/AnimationToolkit/libsrc/animation)
set(AniTkLib /home/alinen/projects/AnimationToolkit/lib/libanimation-soln.a)
file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} main.cpp)
message(WARNING "ANIMATION "  ${AniTk})
target_include_directories(${example_name} PUBLIC ${AniTk})
target_link_libraries(${example_name} PUBLIC ${required_libraries} ${AniTkLib})
