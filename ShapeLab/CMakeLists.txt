cmake_minimum_required(VERSION 3.0.0 )
project(ShapeLab)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)

# find_package(IntelDPCPP REQUIRED)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5OpenGL)
find_package(Qt5Widgets)

set(src main.cpp
	MainWindow.cpp
	DeformTet.cpp
	Fabrication.cpp
	FileIO.cpp
	SupportGeneration.cpp
	heatMethod.cpp
	heatmethodfield.cpp
	IsoLayerGeneration.cpp
	PrincipleStressField.cpp
	toolpathGeneration.cpp
	GcodeGeneration.cpp
	Graph.cpp)
set(headers MainWindow.h
	DeformTet.h
	Fabrication.h
	FileIO.h
	GcodeGeneration.h
	Graph.h
	heatMethod.h
	heatmethodfield.h
	IsoLayerGeneration.h
	PrincipleStressField.h
	SupportGeneration.h
	toolpathGeneration.h)
set(resources ShapeLab.qrc)
set(forms ./MainWindow.ui)

add_executable(ShapeLab ${src} ${headers} ${resources} ${forms})

target_include_directories(ShapeLab PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:.>)
target_include_directories(ShapeLab PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ThirdPartyDependence/eigen3> $<INSTALL_INTERFACE:../ThirdPartyDependence/eigen3>)
target_include_directories(ShapeLab PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ThirdPartyDependence> $<INSTALL_INTERFACE:../ThirdPartyDependence>)
target_include_directories(ShapeLab PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ThirdPartyDependence/glut> $<INSTALL_INTERFACE:../ThirdPartyDependence/glut>)
target_include_directories(ShapeLab PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../QMeshLib> $<INSTALL_INTERFACE:../QMeshLib>)
target_include_directories(ShapeLab PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../GLKLib> $<INSTALL_INTERFACE:../GLKLib>)

target_link_libraries(ShapeLab PRIVATE Qt5::Gui Qt5::Core Qt5::OpenGL Qt5::Widgets)
target_link_libraries(ShapeLab PRIVATE QMeshLib GLKLib)
target_link_libraries(ShapeLab PRIVATE MKL::MKL)
# OpenMP::OpenMP_CXX is an imported target with INTERFACE_**
target_link_libraries(ShapeLab PRIVATE OpenMP::OpenMP_CXX)
link_directories(L${MKLROOT}/lib/intel64)
target_link_libraries(ShapeLab PRIVATE -lpthread -lm -ldl)


set_target_properties(ShapeLab PROPERTIES DEBUG_POSTFIX "d")
install(TARGETS ShapeLab DESTINATION Bin)
