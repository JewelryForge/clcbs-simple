cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)

project(clcbs_driving)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(
        catkin REQUIRED COMPONENTS
        roscpp std_msgs nav_msgs tf
)
catkin_package(
        CATKIN_DEPENDS roscpp std_msgs nav_msgs tf
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(lib src/CarModel.cpp src/LocalPlanner.cpp src/GlobalPlanner.cpp src/PlanVisualizer.cpp src/Angle.cpp)

add_executable(tracking src/main.cpp)

target_link_libraries(tracking lib OsqpEigen::OsqpEigen ${catkin_LIBRARIES} ${Boost_LIBRARIES})