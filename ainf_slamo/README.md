# ainf_slamo

Componente de prueba para **Active Inference** con un robot simulado (Webots) y un LiDAR 3D.  
La idea es estimar/refinar la **pose 2D del robot** dentro de una habitación conocida minimizando
una pérdida basada en la **Signed Distance Function (SDF)** del modelo de la sala.

La interfaz muestra en la banda superior el **FPS del bucle `compute()`** y el valor final de
**SDF (loss)** tras cada optimización.

```
Estado (robot-room): [width, length, x, y, phi]
  width/length : dimensiones de la sala (fijas por ahora)
  x, y, phi   : pose del robot respecto al centro de la sala (optimizadas)
```

---

## Dependencies

The following dependencies are required to build and run **ainf_slamo**.  
Ensure they are installed and properly configured before proceeding.

### 1. System packages

```bash
sudo apt install build-essential cmake git unzip wget \
    qt6-base-dev qt6-declarative-dev qt6-scxml-dev \
    libqt6statemachineqml6 libqt6statemachine6
```

### 2. libQGLViewer (Qt6)

```bash
mkdir -p ~/software
git clone https://github.com/GillesDebunne/libQGLViewer.git ~/software/libQGLViewer
cd ~/software/libQGLViewer
qmake6 *.pro && make -j12 && sudo make install && sudo ldconfig
cd -
```

### 3. toml++

Required for the `.toml` configuration format:

```bash
git clone https://github.com/marzer/tomlplusplus.git ~/software/tomlplusplus
cd ~/software/tomlplusplus
cmake -B build && sudo make install -C build -j12
cd -
```

### 4. LibTorch (C++ PyTorch)

LibTorch is the C++ distribution of PyTorch and is required for the
Active Inference / SDF optimisation pipeline.

> Check <https://pytorch.org/get-started/locally/> for the latest stable URL
> matching your CUDA version.

**Option A – CPU-only** *(recommended for development)*

```bash
cd ~/software
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.2.2%2Bcpu.zip \
     -O libtorch-cpu.zip
unzip libtorch-cpu.zip   # extracts to ~/software/libtorch/
rm libtorch-cpu.zip
```

**Option B – CUDA 12.1** *(GPU acceleration)*

```bash
cd ~/software
wget https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.2.2%2Bcu121.zip \
     -O libtorch-cuda.zip
unzip libtorch-cuda.zip  # extracts to ~/software/libtorch/
rm libtorch-cuda.zip
```

**Register the shared libraries:**

```bash
echo "/home/$USER/software/libtorch/lib" | sudo tee /etc/ld.so.conf.d/libtorch.conf
sudo ldconfig
```

**Point CMake to LibTorch** (add to your shell profile or pass as a flag):

```bash
# Shell profile (~/.bashrc or ~/.zshrc)
export Torch_DIR=~/software/libtorch/share/cmake/Torch

# Or pass directly at configure time:
cmake -B build -DTorch_DIR=~/software/libtorch/share/cmake/Torch
make -C build -j12
```

---

## Configuration parameters

Like any other RoboComp component, **ainf_slamo** requires a configuration file to start.  
Example files are provided in `etc/config` and `etc/config.toml`.

---

## Starting the component

To avoid overwriting the repository config on `git pull`, copy it first:

```bash
cd <ainf_slamo's path>
cp etc/config etc/yourConfig
```

Then compile and run:

```bash
cmake -B build && make -C build -j12   # Compile
bin/ainf_slamo etc/yourConfig          # Run
```

---
---

# Developer Notes

This section explains how to work with the generated code of **ainf_slamo**,
including what can be modified and how to use key features.

## Editable Files

You can freely edit the following files:

- `etc/*` – Configuration files
- `src/*` – Component logic and implementation
- `README.md` – Documentation

> The `generated/` folder contains auto-generated files.  
> **Do not edit them directly** — they will be overwritten every time the
> component is regenerated with RoboComp.

---

## ConfigLoader

`ConfigLoader` simplifies fetching configuration parameters via the `get<>()` method:

```cpp
// Syntax
type variable = this->configLoader.get<type>("ParameterName");

// Example
int computePeriod = this->configLoader.get<int>("Period.Compute");
```

---

## StateMachine

RoboComp components use a state machine to manage the main execution flow.
The default states are:

| State | When | Purpose |
|-------|------|---------|
| **Initialize** | Once at startup | Parameter init, device setup, constant calculation |
| **Compute** | Cyclic | Main functional logic. Call `goToEmergency()` if needed |
| **Emergency** | Cyclic during emergencies | Handle the emergency. Call `goToRestore()` when resolved |
| **Restore** | Once after emergency | Restore state, then transitions back to Compute automatically |

### Getting and setting state periods

```cpp
int currentPeriod = getPeriod("Compute");    // Get current Compute period
setPeriod("Compute", currentPeriod * 0.5);  // Set Compute period to half
```

### Creating custom states

**1. Define the state** using `GRAFCETStep`:

```cpp
states["CustomState"] = std::make_unique<GRAFCETStep>(
    "CustomState", period,
    std::bind(&SpecificWorker::customLoop,  this),  // Cyclic function
    std::bind(&SpecificWorker::customEnter, this),  // On-enter function
    std::bind(&SpecificWorker::customExit,  this)); // On-exit function
```

**2. Define transitions** with `addTransition`:

```cpp
// Syntax
states[srcState]->addTransition(originOfSignal, signal, dstState);

// Examples
states["CustomState"]->addTransition(
    states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
states["Compute"]->addTransition(
    this, SIGNAL(customSignal()), states["CustomState"].get());
```

**3. Register the state** in the state machine:

```cpp
statemachine.addState(states["CustomState"].get());
```

---

## Hibernation Flag

The `#define HIBERNATION_ENABLED` flag in `specificworker.h` activates hibernation mode.

- If no method calls are received within **5 seconds**, the execution frequency drops to **500 ms**.
- Once a method call is received, the period is restored to its original value.
- Default hibernation monitoring runs every **500 ms**.

---

## Changes Introduced in the New Code Generator

- Deprecated classes removed: `CommonBehavior`, `InnerModel`, `AGM`, `Monitors`, `src/config.h`.
- Configuration parsing replaced with `ConfigLoader` (supports both `.toml` and legacy formats).
- Skeleton code split into `generated/` (non-editable) and `src/` (editable).
- Component period is now configurable in the configuration file.
- State machine integrated with predefined states: `Initialize`, `Compute`, `Emergency`, `Restore`.
- With the `dsr` option, `G` is generated in `GenericWorker`. To use `dsrviewer` you need
  `Qt GUI (QMainWindow)` and the `dsr` option enabled in the **CDSL**.
- Strings in the legacy config must now be enclosed in double quotes (`""`).

---

## Adapting Old Components

1. **Config file** – Add `Period.Compute` and `Period.Emergency`; replace Endpoints/Proxies with their names.
2. **CMakeLists** – Merge the new `src/CMakeLists.txt` with the old `CMakeListsSpecific`.
3. **`specificworker.h`**:
   - Add `HIBERNATION_ENABLED`.
   - Update the constructor signature.
   - Replace `setParams` with state definitions.
4. **`specificworker.cpp`**:
   - Refactor the constructor entirely.
   - Move `setParams` logic to the `initialize` state using `ConfigLoader.get<>()`.
   - Remove the old timer/period logic; replace with `getPeriod()` / `setPeriod()`.
   - Add `Emergency` and `Restore` state functions.
   - Add hibernation guard to implements/publish functions:
     ```cpp
     #ifdef HIBERNATION_ENABLED
         hibernation = true;
     #endif
     ```
5. **Configuration strings** – Enclose all strings in the legacy `config` in double quotes (`""`).
6. **DSR option** – `G` is generated in `GenericWorker`. To use `dsrviewer`, integrate
   `Qt GUI (QMainWindow)` and enable `dsr` in the **CDSL**.

---

## Generated Code

When the component is generated, a `generated/` folder is created with non-editable files.  
You can safely delete everything in `src/` **except**:

- `src/specificworker.h`
- `src/specificworker.cpp`
- `src/CMakeLists.txt`
- `src/mainUI.ui`
- `README.md`
- `etc/config`
- `etc/config.toml`
- Your custom classes
