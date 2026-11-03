# Sega Saturn CD block emulator, for a CMake project that adds this directory: the static library
# target cdblock (alias cdblock::cdblock), its include directory public. The Makefile builds the
# shared library and runs the tests.

cmake_minimum_required(VERSION 3.16)
project(cdblock LANGUAGES CXX)

file(GLOB CDBLOCK_SOURCES CONFIGURE_DEPENDS src/*.cpp src/*/*.cpp)
add_library(cdblock STATIC ${CDBLOCK_SOURCES})
add_library(cdblock::cdblock ALIAS cdblock)
target_include_directories(cdblock PUBLIC include)
target_compile_features(cdblock PRIVATE cxx_std_20)
set_target_properties(cdblock PROPERTIES CXX_EXTENSIONS OFF)
