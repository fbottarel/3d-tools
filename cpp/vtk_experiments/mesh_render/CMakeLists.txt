cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

project(mesh_render_basic)

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

if (NOT VTK_FOUND)
  message("Skipping ${PROJECT_NAME}: ${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif()
message (STATUS "VTK_VERSION: ${VTK_VERSION}")
if (VTK_VERSION VERSION_LESS "8.90.0")
  # old system
  include(${VTK_USE_FILE})
  add_executable(mesh_render_bunny MACOSX_BUNDLE mesh_render_bunny.cpp )
  target_link_libraries(mesh_render_bunny PRIVATE ${VTK_LIBRARIES})
else ()
  # include all components
  add_executable(mesh_render_bunny MACOSX_BUNDLE mesh_render_bunny.cpp )
  target_link_libraries(mesh_render_bunny PRIVATE ${VTK_LIBRARIES})
  # vtk_module_autoinit is needed
  vtk_module_autoinit(
    TARGETS mesh_render_bunny
    MODULES ${VTK_LIBRARIES}
    )
endif ()