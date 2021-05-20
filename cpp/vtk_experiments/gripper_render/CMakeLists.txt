cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

project(gripper_render)

# Find the urdfdom dependency

find_package(urdfdom REQUIRED)

# Find the eigen dependency

find_package(Eigen3 REQUIRED)

# Find the liburdf dependency
# find_package(urdf REQUIRED)

# Find VTK modules
find_package(VTK COMPONENTS
  vtkCommonColor
  vtkCommonCore
  vtkCommonDataModel
  vtkFiltersGeneral
  vtkFiltersSources
  vtkIOGeometry
  vtkIOLegacy
  vtkIOPLY
  vtkIOXML
  vtkInteractionStyle
  vtkRenderingAnnotation
  vtkRenderingContextOpenGL2
  vtkRenderingCore
  vtkRenderingFreeType
  vtkRenderingGL2PSOpenGL2
  vtkRenderingOpenGL2
  QUIET
)
message (STATUS "VTK_VERSION: ${VTK_VERSION}")

# Process the executables
include(${VTK_USE_FILE})
include_directories(include)
add_subdirectory(src)