cmake_minimum_required(VERSION 3.15.0)

project(Kalligraph)
set(CMAKE_OSX_DEPLOYMENT_TARGET 10.12)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(KG_BUILD_LOOP_BLINN "Build with the Loop Blinn triangle mesh generator" ON)

set(SOURCES )
set(HEADERS )

if(${KG_BUILD_LOOP_BLINN})
	set(SOURCES ${SOURCES}
		Sources/KGCommon.cpp
		Sources/KGTriangulatorBruteForce.cpp
		Sources/KGMeshGeneratorLoopBlinn.cpp)

	set(HEADERS ${HEADERS}
		Sources/KGCommon.h
		Sources/KGTriangulatorBruteForce.h
		Sources/KGMeshGeneratorLoopBlinn.h)
endif()

add_library(Kalligraph STATIC "${SOURCES}" "${HEADERS}")
target_include_directories(Kalligraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Sources)
