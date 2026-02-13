# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/grid.cpp
  ../src/grid_esdf.cpp
  ../src/mrpt_map_loader.cpp
  ../src/localizer.cpp
    ..src/mppi_controller.cpp
)
# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/grid.h
  ../src/grid_esdf.h
  ../src/mrpt_map_loader.h
  ../src/localizer.h
  ../src/mppi_controller.h
  ${ROBOCOMP}/abstract_graphic_viewer/abstract_graphic_viewer.h
)


LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O3 -fmax-errors=1 -fno-char8_t)

