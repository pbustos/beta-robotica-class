# Sources set
# Obtener la variable de entorno y guardarla en una variable de CMake
SET(ROBOCOMP $ENV{ROBOCOMP})

# Verificar que la ruta no esté vacía para evitar errores crípticos
IF(NOT ROBOCOMP)
    MESSAGE(FATAL_ERROR "La variable de entorno ROBOCOMP no está definida. Por favor, haz 'export ROBOCOMP=/ruta/a/robocomp' o añádela a tu .bashrc")
ENDIF()


LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/mppi_controller.cpp

)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/mppi_controller.h

)


LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

