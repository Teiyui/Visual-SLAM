include_directories( "/usr/include/eigen3" )

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries( draw_trajectory ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})

