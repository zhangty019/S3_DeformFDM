# Install script for directory: C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/out/install/x64-Debug")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/Cholesky"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/CholmodSupport"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/Core"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/Dense"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/Eigen"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/Eigenvalues"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/Geometry"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/Householder"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/IterativeLinearSolvers"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/Jacobi"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/KLUSupport"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/LU"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/MetisSupport"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/OrderingMethods"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/PaStiXSupport"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/PardisoSupport"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/QR"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/QtAlignedMalloc"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/SPQRSupport"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/SVD"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/Sparse"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/SparseCholesky"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/SparseCore"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/SparseLU"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/SparseQR"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/StdDeque"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/StdList"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/StdVector"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/SuperLUSupport"
    "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/UmfPackSupport"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "C:/Users/zhang/Documents/My documents/MeshExample/ShapeLabEmpty/ShapeLabEmpty/eigen3/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

