find_package(Qt6 COMPONENTS Xml PrintSupport REQUIRED)
find_package(Qt6 COMPONENTS 3DCore 3DRender 3DExtras QUIET)

# # Find PyTorch
# # Set LibTorch path
# set(CMAKE_PREFIX_PATH "/home/robocomp/software/libtorch")

# # ---- Torch / CUDA handling ----
# # Some libtorch distributions try to enable CUDA during find_package(Torch).
# # If CUDA isn't available/configured, CMake errors out with:
# #   CMAKE_CUDA_ARCHITECTURES must be non-empty if set.
# # We default to CPU-only.
# set(USE_CUDA OFF CACHE BOOL "Disable CUDA for this component" FORCE)
# set(CAFFE2_USE_CUDA OFF CACHE BOOL "Disable Caffe2 CUDA" FORCE)
# set(CMAKE_DISABLE_FIND_PACKAGE_CUDA OFF CACHE BOOL "Allow CUDA detection for CUDA-enabled libtorch" FORCE)
# set(CUDAToolkit_ROOT "/usr/local/cuda" CACHE PATH "CUDA toolkit root" FORCE)
# set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "CUDA compiler" FORCE)
# # PyTorch ignores CMAKE_CUDA_ARCHITECTURES and expects TORCH_CUDA_ARCH_LIST.
# # Keep a single source of truth to avoid CMake warning noise from Caffe2.
# set(TORCH_CUDA_ARCH_LIST "12.0" CACHE STRING "CUDA arch list for LibTorch" FORCE)

# Find PyTorch
# Set LibTorch path
set(CMAKE_PREFIX_PATH "/home/robocomp/software/libtorch")

# ---- Torch / CUDA handling ----
# Some libtorch distributions try to enable CUDA during find_package(Torch).
# If CUDA isn't available/configured, CMake errors out with:
#   CMAKE_CUDA_ARCHITECTURES must be non-empty if set.
# We default to CPU-only.
set(USE_CUDA ON CACHE BOOL "Enable CUDA for this component" FORCE)
set(CAFFE2_USE_CUDA ON CACHE BOOL "Enable Caffe2 CUDA" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_CUDA OFF CACHE BOOL "Allow CUDA detection for CUDA-enabled libtorch" FORCE)

# Try nvcc in known locations first.
find_program(AINFO_NVCC
  NAMES nvcc
  PATHS
    /usr/bin
    /usr/local/cuda-12.8/bin
  NO_DEFAULT_PATH
)

# Fallback to PATH lookup if not found in explicit locations.
if(NOT AINFO_NVCC)
  find_program(AINFO_NVCC NAMES nvcc)
endif()

if(NOT AINFO_NVCC)
  message(FATAL_ERROR "nvcc not found. Checked /usr/bin and /usr/local/cuda-12.8/bin, then PATH.")
endif()

set(CMAKE_CUDA_COMPILER "${AINFO_NVCC}" CACHE FILEPATH "CUDA compiler" FORCE)

# Infer toolkit root from the compiler path.
get_filename_component(AINFO_NVCC_DIR "${AINFO_NVCC}" DIRECTORY)
if(AINFO_NVCC_DIR STREQUAL "/usr/local/cuda-12.8/bin")
  set(CUDAToolkit_ROOT "/usr/local/cuda-12.8" CACHE PATH "CUDA toolkit root" FORCE)
else()
  set(CUDAToolkit_ROOT "/usr" CACHE PATH "CUDA toolkit root" FORCE)
endif()

# Prefer automatic architecture detection on the local GPU.
set(CMAKE_CUDA_ARCHITECTURES native CACHE STRING "CUDA architectures" FORCE)
if(NOT CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES "86" CACHE STRING "CUDA architectures fallback" FORCE)
endif()

# PyTorch uses TORCH_CUDA_ARCH_LIST (dot-separated). Keep it aligned.
if(CMAKE_CUDA_ARCHITECTURES STREQUAL "native")
  # Let PyTorch infer from the local environment when using native.
  set(TORCH_CUDA_ARCH_LIST "" CACHE STRING "CUDA architectures for LibTorch" FORCE)
elseif(CMAKE_CUDA_ARCHITECTURES STREQUAL "86")
  set(TORCH_CUDA_ARCH_LIST "8.6" CACHE STRING "CUDA architectures for LibTorch" FORCE)
else()
  set(TORCH_CUDA_ARCH_LIST "${CMAKE_CUDA_ARCHITECTURES}" CACHE STRING "CUDA architectures for LibTorch" FORCE)
endif()

# 1. Enable CUDA language so CMake initializes detection scripts
enable_language(CUDA)


find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/specificworker_episode_metrics.cpp
  ../src/em_manager.cpp
  ../src/layout_manager.cpp
  ../src/specificworker_grounding.cpp
  ../src/specificworker_layout_io.cpp
  ../src/specificworker_visualization.cpp
  ../src/specificworker_navigation.cpp
  ../src/specificworker_sensing.cpp
  ../src/viewer_2d.cpp
  ../src/room_concept_ai.cpp
  ../src/room_model.cpp
  ../src/episodic_memory.cpp
  ../src/pointcloud_center_estimator.cpp
  ../src/polygon_path_planner.cpp
  ../src/trajectory_controller.cpp
  ../src/scene_tree_panel.cpp
  ../src/camera_viewer.cpp
  ../src/object_palette_panel.cpp
  ../src/mesh_sdf_optimizer.cpp
  ../src/object_ownership_em.cpp
  ../src/scene_graph_adapter.cpp
  ../src/scene_graph_model.cpp
  ../src/object_models/table_analytic_model.cpp
  ../src/object_models/chair_analytic_model.cpp
  ../src/object_models/bench_analytic_model.cpp
  ../src/object_models/pot_analytic_model.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/em_manager.h
  ../src/layout_manager.h
  ../src/viewer_2d.h
  ../src/room_concept_ai.h
  ../src/room_model.h
  ../src/episodic_memory.h
  ../src/polygon_path_planner.h
  ../src/trajectory_controller.h
  ../src/scene_tree_panel.h
  ../src/camera_viewer.h
  ../src/object_palette_panel.h
  ../src/mesh_sdf_optimizer.h
  ../src/object_ownership_em.h
  ../src/furniture_types.h
  ../src/scene_graph_adapter.h
  ../src/scene_graph_model.h
  ../src/object_models/table_analytic_model.h
  ../src/object_models/chair_analytic_model.h
  ../src/object_models/bench_analytic_model.h
  ../src/object_models/pot_analytic_model.h
)


LIST(APPEND LIBS ${LIBS} "${TORCH_LIBRARIES}" tbb Qt6PrintSupport Qt6Xml
            Eigen3::Eigen Boost::system Boost::filesystem )

if(TARGET Qt6::3DCore AND TARGET Qt6::3DRender AND TARGET Qt6::3DExtras)
  LIST(APPEND SOURCES ../src/viewer_3d.cpp)
  LIST(APPEND HEADERS ../src/viewer_3d.h)
  LIST(APPEND LIBS Qt6::3DCore Qt6::3DRender Qt6::3DExtras)
else()
  message(WARNING "Qt3D targets not found; building without 3D viewer")
endif()

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O3 -fmax-errors=1 -fno-char8_t)