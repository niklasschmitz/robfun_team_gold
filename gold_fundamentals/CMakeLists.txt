# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(gold_fundamentals)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs create_fundamentals message_generation)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(
        FILES
        Cell.msg
        Grid.msg
        Pose.msg
        Row.msg
)
#uncomment if you have defined services
add_service_files(
        FILES
        ExecutePlan.srv
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)

#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES std_msgs create_fundamentals
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
        DEPENDS python-serial
        CATKIN_DEPENDS roscpp rospy sensor_msgs create_fundamentals message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} src/utils)

set(
        UTILS
        src/utils/Robot.cpp src/utils/Robot.h
        src/utils/tools.cpp src/utils/tools.h
        src/utils/PID.cpp src/utils/PID.h
        src/utils/GridPerceptor.cpp src/utils/GridPerceptor.h
        src/utils/Maze.cpp src/utils/Maze.h
        src/utils/DiscreteLocalizer.cpp src/utils/DiscreteLocalizer.h
        src/utils/geometry.cpp src/utils/geometry.h
        src/utils/probability.cpp src/utils/probability.h
)

# Assignment 1
add_executable(wanderer src/wanderer.cpp ${UTILS})
target_link_libraries(wanderer ${catkin_LIBRARIES})
add_dependencies(wanderer create_fundamentals_generate_messages_cpp gold_fundamentals_generate_messages_cpp)

add_executable(square_no_sensors src/square_no_sensors.cpp ${UTILS})
target_link_libraries(square_no_sensors ${catkin_LIBRARIES})
add_dependencies(square_no_sensors create_fundamentals_generate_messages_cpp gold_fundamentals_generate_messages_cpp)

add_executable(square_with_encoders src/square_with_encoders.cpp ${UTILS})
target_link_libraries(square_with_encoders ${catkin_LIBRARIES})
add_dependencies(square_with_encoders create_fundamentals_generate_messages_cpp gold_fundamentals_generate_messages_cpp)

# Assignment 2
add_executable(align_robot src/align_robot.cpp ${UTILS})
target_link_libraries(align_robot ${catkin_LIBRARIES})
add_dependencies(align_robot create_fundamentals_generate_messages_cpp gold_fundamentals_generate_messages_cpp)

add_executable(plan_executer src/plan_executer.cpp ${UTILS})
target_link_libraries(plan_executer ${catkin_LIBRARIES})
add_dependencies(plan_executer create_fundamentals_generate_messages_cpp gold_fundamentals_generate_messages_cpp)

add_executable(smooth_executer src/smooth_executer.cpp ${UTILS})
target_link_libraries(smooth_executer ${catkin_LIBRARIES})
add_dependencies(smooth_executer create_fundamentals_generate_messages_cpp gold_fundamentals_generate_messages_cpp)

# Assignment 3
