project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/Pid.cpp src/Twiddler.cpp src/PidController.cpp src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(pid ${sources})

target_link_libraries(pid z ssl uv uWS)

# Makes boolean 'test' available
option(test "Build all tests" OFF)
################################
# Testing
################################
if (test)
  # Enable ExternalProject CMake module
  include(ExternalProject)

  # Download and install GoogleTest
  ExternalProject_Add(
    gtest
    URL https://github.com/google/googletest/archive/release-1.8.0.tar.gz
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
    # Disable install step
    INSTALL_COMMAND ""
  )

  # Get GTest source and binary directories from CMake project
  ExternalProject_Get_Property(gtest source_dir binary_dir)

  # Create a libgtest target to be used as a dependency by test programs
  add_library(libgtest IMPORTED STATIC GLOBAL)
  add_dependencies(libgtest gtest)

  # Set libgtest properties
  set_target_properties(libgtest PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest.a"
  )

  # Create a libgmock target to be used as a dependency by test programs
  add_library(libgmock IMPORTED STATIC GLOBAL)
  add_dependencies(libgmock gtest)

  # Set libgmock properties
  set_target_properties(libgmock PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/libgmock.a"
  )

  include_directories("${source_dir}/googletest/include"
                      "${source_dir}/googlemock/include")

  # Add components under test
  add_library(twiddler_lib src/Twiddler.cpp)
  add_library(pid_lib src/Pid.cpp)
  add_library(pid_controller_lib src/PidController.cpp)

  target_link_libraries(pid twiddler_lib)
  target_link_libraries(pid pid_lib)
  target_link_libraries(pid pid_controller_lib)

  enable_testing()

  ##############
  # Unit Tests
  ##############
  add_executable(test_twiddler test/TestTwiddler.cpp)
  add_executable(test_pid test/TestPid.cpp)
  add_executable(test_pid_controller test/TestPidController.cpp)

  # Standard linking to gtest stuff
  target_link_libraries(test_twiddler libgtest libgmock)
  target_link_libraries(test_pid libgtest)
  target_link_libraries(test_pid_controller libgtest libgmock)

  # Extra linking for the project
  target_link_libraries(test_twiddler twiddler_lib)
  target_link_libraries(test_pid pid_lib)
  target_link_libraries(test_pid_controller pid_controller_lib pid_lib twiddler_lib)

  # Make tests running through 'make test'
  add_test(NAME test_twiddler COMMAND test_twiddler)
  add_test(NAME test_pid COMMAND test_pid)
  add_test(NAME test_pid_controller COMMAND test_pid_controller)
endif()
