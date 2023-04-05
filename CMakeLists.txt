cmake_minimum_required(VERSION 3.23.0 )
project(ShapeLab)

execute_process(COMMAND '/bin/sh ~/intel/oneapi/setvars.sh')
list(APPEND CMAKE_PREFIX_PATH '~/Qt/5.15.2')
list(APPEND CMAKE_PREFIX_PATH "./ThirdPartyDependence/eigen3/cmake/")

#add_definitions(-DEIGEN_USE_MKL_ALL)
add_definitions(-DEIGEN_USE_MKL_VML)
# add_definitions(-DMKL_LP64)
set(MKL_INTERFACE_LIST intel_ilp64 ${IFACE_TYPE}_lp64)
# set(MKL_INTERFACE_LIST intel_ilp64 ${IFACE_TYPE}_lp64)
# add_definitions(-DMKL_INT=int)
# add_definitions(-DEIGEN_USE_MKL_VML)
set(MKL_INTERFACE_FULL intel_lp64)
add_definitions(-DMKL_INTERFACE=lp64)

find_package(MKL REQUIRED)
find_package(OpenGL)
find_package(GLUT)
include_directories(${MKL_ROOT}/include)
link_directories(${MKLROOT}/lib/intel64)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5Core REQUIRED)

add_subdirectory(QMeshLib)
add_subdirectory(GLKLib)
add_subdirectory(ShapeLab)


