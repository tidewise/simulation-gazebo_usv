find_package(gz-msgs REQUIRED)
gz_msgs_generate_messages(
    TARGET msgs
    PROTO_PACKAGE "gz.gazebo_usv"
    MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
    MSGS_PROTOS
        ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/gazebo_usv/thruster.proto
        ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/gazebo_usv/thrusters.proto
        ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/gazebo_usv/wave.proto
    DEPENDENCIES gz-msgs::gz-msgs
)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(gz-sim-physics-system REQUIRED)
rock_library(gazebo_usv
    SOURCES USVPlugin.cpp Thrusters.cpp Thruster.cpp Rudder.cpp Wind.cpp Wave.cpp DirectForceApplication.cpp Buoyancy.cpp
    HEADERS USVPlugin.hpp Thrusters.hpp Thruster.hpp Rudder.hpp Wind.hpp Wave.hpp DirectForceApplication.hpp Buoyancy.hpp Components.hpp
    ${__boost_deps}
    DEPS_CMAKE gz-sim-physics-system
    DEPS_PKGCONFIG base-types sdformat protobuf)

target_link_libraries(gazebo_usv gazebo_usv-msgs)
