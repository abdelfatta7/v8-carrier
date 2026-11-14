# Native (host) build of the hardware-independent firmware libraries and their
# tests. This is the path that runs anywhere with a C++17 compiler:
#
#   cd firmware && cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
#
# The ESP32-S3 target itself is built with PlatformIO (platformio.ini); the
# same test sources also run there with `pio test -e native`.
cmake_minimum_required(VERSION 3.16)
project(v8carrier_firmware_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

enable_testing()

# One static library per lib/<module>, no Arduino anywhere.
set(V8C_MODULES pid odometry protocol safety vacuum_button miclevel)
foreach(mod ${V8C_MODULES})
  file(GLOB mod_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/lib/${mod}/*.cpp)
  add_library(v8c_${mod} STATIC ${mod_sources})
  target_include_directories(v8c_${mod} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/${mod})
endforeach()

# One test executable per test/test_<module>/ directory.
foreach(mod ${V8C_MODULES})
  set(exe test_${mod})
  add_executable(${exe} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_${mod}/test_${mod}.cpp)
  target_link_libraries(${exe} PRIVATE v8c_${mod})
  target_include_directories(${exe} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
endforeach()

add_test(NAME pid COMMAND test_pid)
add_test(NAME odometry COMMAND test_odometry)
add_test(NAME safety COMMAND test_safety)
add_test(NAME vacuum_button COMMAND test_vacuum_button)
add_test(NAME miclevel COMMAND test_miclevel)

# The protocol test also writes the cross-language test vectors. A second test
# checks that the committed docs/protocol_vectors.json matches what the C++
# encoder produces today, so the Python side (pytest) is checked against the
# real byte layout and the file cannot silently go stale.
set(V8C_VECTORS_BUILD ${CMAKE_BINARY_DIR}/protocol_vectors.json)
set(V8C_VECTORS_DOCS ${CMAKE_CURRENT_SOURCE_DIR}/../docs/protocol_vectors.json)
add_test(NAME protocol COMMAND test_protocol ${V8C_VECTORS_BUILD})
add_test(NAME protocol_vectors_match_docs
         COMMAND ${CMAKE_COMMAND} -E compare_files ${V8C_VECTORS_BUILD} ${V8C_VECTORS_DOCS})
set_tests_properties(protocol PROPERTIES FIXTURES_SETUP vectors)
set_tests_properties(protocol_vectors_match_docs PROPERTIES FIXTURES_REQUIRED vectors)

# `cmake --build build --target update_protocol_vectors` regenerates the
# committed file after a deliberate protocol change.
add_custom_target(update_protocol_vectors
  COMMAND test_protocol ${V8C_VECTORS_DOCS}
  DEPENDS test_protocol
  COMMENT "Regenerating docs/protocol_vectors.json from the C++ encoder")
