cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Find VTK
find_package(VTK)
if(VTK_FOUND)
 include (${VTK_USE_FILE})
endif()

pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)
FIND_PACKAGE ( OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osg)

INCLUDE_DIRECTORIES(${OPENSCENEGRAPH_INCLUDE_DIRS} ${LIBXML++_INCLUDE_DIRS})
LINK_DIRECTORIES(${LIBXML++_LIBRARY_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(example src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
 rosbuild_add_executable(save_cloud src/save_cloud.cpp)

 #rosbuild_add_executable(cluster_segmentation src/cluster_segmentation.cpp)
 #rosbuild_add_executable(visp src/visp.cpp)

# rosbuild_add_executable(registration2 src/registration2.cpp)  
#  rosbuild_add_executable(example2 src/example2.cpp)
  rosbuild_add_executable(cylinder src/cylinder.cpp)
#  rosbuild_add_executable(cylinder_reconstruction src/cylinder_reconstruction.cpp)
#  rosbuild_add_executable(simple_viewer src/simple_viewer.cpp)
#  rosbuild_add_executable(acum src/acum.cpp)
#  rosbuild_add_executable(incremental src/incremental.cpp)
#  rosbuild_add_executable(viewer src/viewer.cpp)
  rosbuild_add_executable(arm5e_pc_grasp_planning src/arm5e_pc_grasp_planning.cpp)

  rosbuild_add_executable(grasp_exec src/grasp_exec.cpp)

# target_link_libraries (viewer ${PCL_LIBRARIES} vtkHybrid vtkRendering) 
# target_link_libraries (simple_viewer ${PCL_LIBRARIES} vtkHybrid vtkRendering) 
# target_link_libraries (cylinder_reconstruction ${PCL_LIBRARIES} vtkHybrid vtkRendering) 
 target_link_libraries (arm5e_pc_grasp_planning ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES} ${PCL_LIBRARIES} vtkHybrid vtkRendering) 
# target_link_libraries (registration2 vtkHybrid vtkRendering) 

