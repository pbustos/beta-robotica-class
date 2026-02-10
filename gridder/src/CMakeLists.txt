# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/grid.cpp
  ../src/grid_esdf.cpp
)
# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/grid.h
  ../src/grid_esdf.h
  ${ROBOCOMP}/abstract_graphic_viewer/abstract_graphic_viewer.h
)


LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O3 -fmax-errors=1 -fno-char8_t)

