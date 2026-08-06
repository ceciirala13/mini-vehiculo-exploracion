# Top-level CMakeLists.txt — wrapper for the ESP-IDF extension.
#
# The VS Code workspace root is BNO086/, but the actual ESP-IDF project
# sources live in BNO086/bno086_test/main/.  We tell the build system
# where to find the main component so "idf.py build" works from here.
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/bno086_test/main")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bno086_test)
