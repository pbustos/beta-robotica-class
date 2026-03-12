cmake_minimum_required( VERSION 3.10 )
PROJECT( ainf_slamo )

IF ( "$ENV{ROBOCOMP}" STREQUAL "")
  MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /home/robocomp/robocomp")
  SET (ENV{ROBOCOMP} "/home/robocomp/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

SET(RC_COMPONENT_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component base path" FORCE)
SET(RC_COMPONENT_DEVEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "component binary path" FORCE)
#SET(RC_COMPONENT_DEVEL_PATH "${WORKSPACE_PATH}/devel"  CACHE STRING "component binary path")
SET(RC_COMPONENT_INSTALL_PATH $ENV{ROBOCOMP} CACHE STRING "component install path")

#commented as now devel is merged into source space
#get_filename_component( COMPONENT_NAME ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
#STRING(REPLACE "${COMPONENT_NAME}/" " " COMPONENT_NAME  ${RC_COMPONENT_PATH})
#STRING(STRIP ${COMPONENT_NAME} COMPONENT_NAME)
#SET(COMPONENT_NAME ${COMPONENT_NAME} CACHE STRING "component name" FORCE)

add_subdirectory(generated)

INSTALL(FILES etc/config DESTINATION ${RC_COMPONENT_INSTALL_PATH}/etc-default/ RENAME ainf_slamo.conf )

# Install runtime mesh assets used by the 3D viewer (bench/chair/monitor/etc.).
# Without these files, only analytic furniture (e.g. tables) remains visible.
INSTALL(DIRECTORY meshes DESTINATION ${RC_COMPONENT_INSTALL_PATH})

# Optional: deploy Qt scene parser plugins (Assimp) so Qt3DRender::QMesh can
# load OBJ files in installations where plugin discovery is incomplete.
find_program(QTPATHS_EXECUTABLE NAMES qtpaths6 qtpaths)
if(QTPATHS_EXECUTABLE)
  execute_process(
    COMMAND ${QTPATHS_EXECUTABLE} --plugin-dir
    OUTPUT_VARIABLE QT_PLUGIN_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

  if(EXISTS "${QT_PLUGIN_DIR}/sceneparsers")
    INSTALL(DIRECTORY "${QT_PLUGIN_DIR}/sceneparsers"
            DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/plugins)
    message(STATUS "Installing Qt sceneparsers from: ${QT_PLUGIN_DIR}/sceneparsers")
  else()
    message(WARNING "Qt sceneparsers plugin directory not found under: ${QT_PLUGIN_DIR}")
  endif()
else()
  message(WARNING "qtpaths not found; skipping Qt sceneparsers plugin install")
endif()

#ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
#ADD_CUSTOM_TARGET(installdoc mkdir -p ${RC_COMPONENT_INSTALL_PATH}/doc COMMAND cp -R doc/html ${RC_COMPONENT_INSTALL_PATH}/doc/ainf_slamo )
