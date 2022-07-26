
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(rs_driver_wrapper
    VERSION 0.1.0
    DESCRIPTION "RS Driver wrapper"
    LANGUAGES CXX
)

if (NOT EXISTS "./pybind11")
    execute_process(
        COMMAND "git" "clone" "--recursive" "https://github.com/pybind/pybind11.git"
    )
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# pybind11
add_subdirectory(build/pybind11)
pybind11_add_module(rs_driver_wrapper rs_driver_wrapper.cpp)

# RS Driver
target_include_directories(rs_driver_wrapper
  PUBLIC /usr/local/rs_driver/include
)
find_package(rs_driver REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(Boost REQUIRED COMPONENTS regex)

set_target_properties(rs_driver_wrapper
    PROPERTIES
    VERSION ${PROJECT_VERSION}
)

target_compile_options(rs_driver_wrapper PUBLIC -O2 -Wall)

target_link_libraries(rs_driver_wrapper PUBLIC pthread ${rs_driver_LIBRARIES})
