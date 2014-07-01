cmake_minimum_required(VERSION 2.8.3)
project( learn_play )

find_package(catkin REQUIRED
  COMPONENTS
  cv_bridge
  image_transport
  )
find_package(OpenCV REQUIRED)


# generate_dynamic_reconfigure_options(
#   cfg/JointSpringsExample.cfg
#   )

catkin_package(
  CATKIN_DEPENDS roscpp
  )

# Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

## Declare a cpp library
# add_library(baxter_demo
#   src/${PROJECT_NAME}/baxter_demo.cpp
# )

## Declare a cpp executable
add_executable( learn_play src/main.cpp )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(model_tracker baxter_demo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( learn_play ${OpenCV_LIBS} )
target_link_libraries( learn_play ${catkin_LIBRARIES})
