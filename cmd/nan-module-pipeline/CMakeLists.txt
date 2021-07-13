cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pipeline)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")
file(GLOB SOURCE_FILES "main.cpp" "../../src/Pipeline.cpp" "../../src/GeometricFeatures.cpp"  "../../src/Utils.cpp" "../../src/Pipeline.h" "../../src/GeometricFeatures.h" "../../src/Utils.h")
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "" SUFFIX ".node")
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_JS_INC})
target_link_libraries (pipeline ${PCL_LIBRARIES} ${CMAKE_JS_LIB})