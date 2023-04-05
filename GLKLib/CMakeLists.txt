cmake_minimum_required(VERSION 3.0.0 )
project(GLKLib)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5Widgets)
find_package(Qt5OpenGL)
set(src ./GLKGeometry.cpp
	./GLKGraph.cpp
	./GLKHeap.cpp
	./GLKLib.cpp
	./GLKMatrixLib.cpp
	./GLKNearestNeighbor.cpp
	./GLKObList.cpp
	./GLKRational.cpp
	./GLKSparseMatrix.cpp
	./GLKSparseMatrixDirectSolver.cpp
	./GLKSparseMatrixIterativeSolver.cpp
	./InteractiveTool.cpp
	./stdafx.cpp)
set(headers ./GLKCameraTool.h
	./GLKGLList.h
	./GLKGeometry.h
	./GLKGraph.h
	./GLKHeap.h
	./GLKMatrixLib.h
	./GLKNearestNeighbor.h
	./GLKObList.h
	./GLKRational.h
	./GLKSparseMatrix.h
	./GLKSparseMatrixDirectSolver.h
	./GLKSparseMatrixIterativeSolver.h
	./InteractiveTool.h
	./GLKLib.h
	./stdafx.h ../ThirdPartyDependence/io.h ../ThirdPartyDependence/StdAfx.h)
add_library(GLKLib ${src} ${headers})
target_link_libraries(GLKLib PRIVATE Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
target_link_libraries(GLKLib PRIVATE -lglut -lGLU -lGL)
target_compile_options(GLKLib PUBLIC $<TARGET_PROPERTY:MKL::MKL,INTERFACE_COMPILE_OPTIONS>)
target_include_directories(GLKLib PUBLIC $<TARGET_PROPERTY:MKL::MKL,INTERFACE_INCLUDE_DIRECTORIES>)
target_link_libraries(GLKLib PUBLIC $<LINK_ONLY:MKL::MKL>)

target_include_directories(GLKLib PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ThirdPartyDependence/glut> $<INSTALL_INTERFACE:../ThirdPartyDependence/glut>)
target_include_directories(GLKLib PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:.>)
target_compile_definitions(GLKLib PRIVATE  _ENABLE_EXTENDED_ALIGNED_STORAGE)
target_compile_definitions(GLKLib PRIVATE  _UNICODE)
set_target_properties(GLKLib PROPERTIES DEBUG_POSTFIX "d")
install(TARGETS GLKLib DESTINATION Libs)
