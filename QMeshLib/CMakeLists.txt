cmake_minimum_required(VERSION 3.0.0 )
project(QMeshLib)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5OpenGL)
set(src
		../ThirdPartyDependence/PQPLib/BV.cpp
		../ThirdPartyDependence/PQPLib/Build.cpp
		../ThirdPartyDependence/PQPLib/PQP.cpp
		../ThirdPartyDependence/PQPLib/TriDist.cpp
		PolygenMesh.cpp
		QMeshCluster.cpp
		QMeshEdge.cpp
		QMeshFace.cpp
		QMeshNode.cpp
		QMeshPatch.cpp
		QMeshTetra.cpp
		../ThirdPartyDependence/QHullLib/geom.cpp
		../ThirdPartyDependence/QHullLib/geom2.cpp
		../ThirdPartyDependence/QHullLib/global.cpp
		../ThirdPartyDependence/QHullLib/io.cpp
		../ThirdPartyDependence/QHullLib/men.cpp
		../ThirdPartyDependence/QHullLib/merge.cpp
		../ThirdPartyDependence/QHullLib/poly.cpp
		../ThirdPartyDependence/QHullLib/poly2.cpp
		../ThirdPartyDependence/QHullLib/qconvex.cpp
		../ThirdPartyDependence/QHullLib/qhull.cpp
		../ThirdPartyDependence/QHullLib/qset.cpp
		../ThirdPartyDependence/QHullLib/rbox.cpp
		../ThirdPartyDependence/QHullLib/stat.cpp
		../ThirdPartyDependence/QHullLib/user.cpp)
set(headers
		../ThirdPartyDependence/PQPLib/BV.h
		../ThirdPartyDependence/PQPLib/BVTQ.h
		../ThirdPartyDependence/PQPLib/Build.h
		../ThirdPartyDependence/PQPLib/GetTime.h
		../ThirdPartyDependence/PQPLib/MatVec.h
		../ThirdPartyDependence/PQPLib/OBB_Disjoint.h
		../ThirdPartyDependence/PQPLib/PQP.h
		../ThirdPartyDependence/PQPLib/PQP_Compile.h
		../ThirdPartyDependence/PQPLib/PQP_Internal.h
		PolygenMesh.h
		QMeshCluster.h
		QMeshEdge.h
		QMeshFace.h
		QMeshNode.h
		QMeshPatch.h
		QMeshTetra.h
		../ThirdPartyDependence/PQPLib/RectDist.h
		../ThirdPartyDependence/PQPLib/Tri.h
		../ThirdPartyDependence/PQPLib/TriDist.h
		../ThirdPartyDependence/QHullLib/geom.h
		../ThirdPartyDependence/QHullLib/io.h
		../ThirdPartyDependence/QHullLib/mem.h
		../ThirdPartyDependence/QHullLib/merge.h
		../ThirdPartyDependence/QHullLib/poly.h
		../ThirdPartyDependence/QHullLib/qhull.h
		../ThirdPartyDependence/QHullLib/qhull_a.h
		../ThirdPartyDependence/QHullLib/qset.h
		../ThirdPartyDependence/QHullLib/stat.h
		../ThirdPartyDependence/QHullLib/user.h)
add_library(QMeshLib ${src} ${headers})
target_link_libraries(QMeshLib PRIVATE Qt5::Widgets Qt5::Gui Qt5::Core Qt5::OpenGL)
target_link_libraries(QMeshLib PRIVATE -lglut -lGLU -lGL)

target_include_directories(QMeshLib PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ThirdPartyDependence/glut> $<INSTALL_INTERFACE:../ThirdPartyDependence/glut>)
target_compile_definitions(QMeshLib PRIVATE  _UNICODE)
target_compile_definitions(QMeshLib PRIVATE  _ENABLE_EXTENDED_ALIGNED_STORAGE)
set_target_properties(QMeshLib PROPERTIES DEBUG_POSTFIX "d")

install(TARGETS QMeshLib DESTINATION Libs)
