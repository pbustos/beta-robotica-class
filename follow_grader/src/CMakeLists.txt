# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/follow_score.cpp
  ../src/time_series_plot.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/follow_score.h
  ../src/time_series_plot.h
)


# the generated ui header includes custom widgets (time_series_plot.h) from here
include_directories(${CMAKE_CURRENT_LIST_DIR})

LIST(APPEND LIBS ${LIBS})

add_compile_options(
    -O3 
    -march=native 
    -flto=auto
    -fno-plt
)
add_definitions(-DNDEBUG -fmax-errors=1)
add_link_options(-flto=auto)

