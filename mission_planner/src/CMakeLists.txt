# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
        ../src/grid_esdf.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
        ../src/grid_esdf.h
)


LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O3 -fmax-errors=1 -fno-char8_t)

