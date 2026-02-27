find_package(Qt6 COMPONENTS 3DCore 3DRender 3DExtras Xml REQUIRED)

# Find PyTorch
# Set LibTorch path
set(CMAKE_PREFIX_PATH "/home/robocomp/software/libtorch")

# ---- Torch / CUDA handling ----
# Some libtorch distributions try to enable CUDA during find_package(Torch).
# If CUDA isn't available/configured, CMake errors out with:
#   CMAKE_CUDA_ARCHITECTURES must be non-empty if set.
# We default to CPU-only.
set(USE_CUDA OFF CACHE BOOL "Disable CUDA for this component" FORCE)
set(CAFFE2_USE_CUDA OFF CACHE BOOL "Disable Caffe2 CUDA" FORCE)
set(TORCH_CUDA_ARCH_LIST "" CACHE STRING "" FORCE)
if(DEFINED CMAKE_CUDA_ARCHITECTURES AND "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "")
  # Provide a safe default if someone exported it as empty.
  unset(CMAKE_CUDA_ARCHITECTURES CACHE)
endif()

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/room_concept_ai.cpp
  ../src/room_model.cpp
  ../src/pointcloud_center_estimator.cpp


)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/room_concept_ai.h
  ../src/room_model.h
)


LIST(APPEND LIBS ${LIBS} "${TORCH_LIBRARIES}" tbb Qt63DCore Qt63DRender Qt63DExtras Qt6PrintSupport Qt6Xml
            Eigen3::Eigen Boost::system Boost::filesystem )

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O3 -fmax-errors=1 -fno-char8_t)
