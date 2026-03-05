find_package(Qt6 COMPONENTS Xml PrintSupport REQUIRED)
find_package(Qt6 COMPONENTS 3DCore 3DRender 3DExtras QUIET)

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
set(CMAKE_DISABLE_FIND_PACKAGE_CUDA OFF CACHE BOOL "Allow CUDA detection for CUDA-enabled libtorch" FORCE)
set(CUDAToolkit_ROOT "/usr/local/cuda" CACHE PATH "CUDA toolkit root" FORCE)
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "CUDA compiler" FORCE)
# PyTorch ignores CMAKE_CUDA_ARCHITECTURES and expects TORCH_CUDA_ARCH_LIST.
# Keep a single source of truth to avoid CMake warning noise from Caffe2.
set(TORCH_CUDA_ARCH_LIST "12.0" CACHE STRING "CUDA arch list for LibTorch" FORCE)


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
  ../src/polygon_path_planner.cpp
  ../src/trajectory_controller.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/room_concept_ai.h
  ../src/room_model.h
  ../src/polygon_path_planner.h
  ../src/trajectory_controller.h
)


LIST(APPEND LIBS ${LIBS} "${TORCH_LIBRARIES}" tbb Qt6PrintSupport Qt6Xml
            Eigen3::Eigen Boost::system Boost::filesystem )

if(TARGET Qt6::3DCore AND TARGET Qt6::3DRender AND TARGET Qt6::3DExtras)
  LIST(APPEND LIBS Qt63DCore Qt63DRender Qt63DExtras)
endif()

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O3 -fmax-errors=1 -fno-char8_t)