cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(experiments)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_libraries(stdc++fs)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build2")

add_executable (
  experiments
  ../../src/Computation.cpp
  ../../src/Computation.h
  ../../src/Cropper.cpp
  ../../src/Cropper.h
  ../../src/NosetipFinder.cpp
  ../../src/NosetipFinder.h
  ../../src/EyeRightCorner.cpp
  ../../src/EyeRightCorner.h
  ../../src/GeometricFeatures.cpp
  ../../src/GeometricFeatures.h
  ../../src/CloudsLog.cpp
  ../../src/CloudsLog.h
  ../../src/Utils.cpp
  ../../src/Utils.h
  ../../src/Main.cpp
  ../../src/Main.h
  ../../src/Pipeline.cpp
  ../../src/Pipeline.h
  ../../src/PipelineMain.cpp
  ../../src/PipelineMain.h
  main.cpp
  )
target_link_libraries (experiments ${PCL_LIBRARIES})
