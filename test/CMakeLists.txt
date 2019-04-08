cmake_minimum_required(VERSION 3.5.0)
project(etl_unit_tests)

set (CMAKE_CXX_STANDARD 14)

add_definitions(-DETL_DEBUG)


if(NOT UnitTest++_FOUND)
  # Add unittest-cpp as an ExternalProject
  include(cmake/unit-test_external_project.cmake)
  add_unittest_cpp()
else()
  include(${UnitTest++_CONFIG})
  # The CMake install of UnitTest++ puts the include directory to ${CMAKE_INSTALL_PREFIX}/include
  # under the assumptions that clients will consume the headers via
  # #include<UnitTest++/UnitTest++.h>
  # However the tests in etl include the headers directly, e.g.
  # #include "UnitTest++.h"
  # Therefore we update the provide UTPP_INCLUDE_DIRS here
  set(UTPP_INCLUDE_DIRS "${UTPP_INCLUDE_DIRS}/UnitTest++")
endif()
include_directories(${UTPP_INCLUDE_DIRS})

set(TEST_SOURCE_FILES
  main.cpp
  murmurhash3.cpp
  ##  test_algorithm.cpp
  ## test_alignment.cpp
  ## test_array.cpp
  ## test_array_view.cpp
  ## test_array_wrapper.cpp
  ## test_binary.cpp
  ## test_bitset.cpp
  ## test_bloom_filter.cpp
  ## test_bsd_checksum.cpp
  ## test_callback_timer.cpp
  ## test_checksum.cpp
  ## test_compare.cpp
  ## test_constant.cpp
  ## test_container.cpp
  ## test_crc.cpp
  ## test_c_timer_framework.cpp
  ## test_cyclic_value.cpp
  ## test_debounce.cpp
  ## test_deque.cpp
  test_fsm.cpp


  # Compile the source level ecl_timer here as test has provided a ecl_user.h file
  # ${PROJECT_SOURCE_DIR}/../src/c/ecl_timer.c
  )



if (WIN32)
  # test_error_handler.cpp uses windows APIs that assume the microsoft project
  # is setup for unicode support.  Note: This may have adverse effects on client
  # projects and the tests should be updated to be independent of project setup
  ADD_DEFINITIONS(-DUNICODE)
  ADD_DEFINITIONS(-D_UNICODE)
endif()


if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  list(APPEND TEST_SOURCE_FILES "test_atomic_gcc_sync.cpp")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")
endif()
add_executable(etl_tests
  ${TEST_SOURCE_FILES}
  )
##target_link_libraries(etl_tests etl UnitTest++)
target_link_libraries(etl_tests UnitTest++)
target_include_directories(etl_tests
  PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ../include
  )

# Enable the 'make test' CMake target using the executable defined above
add_test(etl_unit_tests etl_tests)

# Since ctest will only show you the results of the single executable
# define a target that will output all of the failing or passing tests
# as they appear from UnitTest++
add_custom_target(test_verbose COMMAND ${CMAKE_CTEST_COMMAND} --verbose)


# Remaining Tests to be implemented:
#1. Enable embedded compile target testing
#   - test_embedded_compile.cpp
