find_package(Boost 1.40 COMPONENTS system REQUIRED)
find_package(Protobuf REQUIRED)

if (TARGET Boost::system)
    list(APPEND __boost_deps DEPS_TARGET Boost::system)
else()
    list(APPEND __boost_deps DEPS_PLAIN Boost)
endif()

protobuf_generate_cpp(msgs_SRCS msgs_HDR msgs.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rock_library(gazebo_usv
    SOURCES USVPlugin.cpp Thrusters.cpp Thruster.cpp Actuators.cpp Utilities.cpp Rudder.cpp Wind.cpp ${msgs_SRCS}
    HEADERS USVPlugin.hpp Thrusters.hpp Thruster.hpp Actuators.hpp Utilities.hpp Rudder.hpp Wind.hpp
    ${__boost_deps}
    DEPS_PKGCONFIG gazebo sdformat protobuf gazebo_underwater_msgs)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/msgs.pb.h
    DESTINATION include/${PROJECT_NAME})

install(FILES msgs.proto
    DESTINATION include/${PROJECT_NAME}/proto)
