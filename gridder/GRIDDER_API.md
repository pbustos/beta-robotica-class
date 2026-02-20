# Gridder API

API completa del componente Gridder. Proporciona servicios de planificación de paths, gestión de grids de ocupación, localización y control de navegación.

## Índice

- [Estructuras de Datos](#estructuras-de-datos)
- [Path Planning](#path-planning)
- [Grid y Mapa](#grid-y-mapa)
- [Localización](#localización)
- [Gestión de Target](#gestión-de-target)
- [Control de Ejecución](#control-de-ejecución)
- [Consulta de Estado](#consulta-de-estado)
- [Diagrama de Estados](#diagrama-de-estados)
- [Ejemplos de Uso](#ejemplos-de-uso)

---

## Estructuras de Datos

### TPoint (struct)

Punto 2D con radio opcional.

```cpp
struct TPoint {
    float x;        // mm - coordenada X
    float y;        // mm - coordenada Y
    float radius;   // mm - radio del objeto (opcional)
};
```

### TPath / TPaths

```cpp
sequence<TPoint> TPath;     // Un path: secuencia de waypoints
sequence<TPath> TPaths;     // Múltiples paths alternativos
```

### TDimensions (struct)

Dimensiones del grid en coordenadas del mundo.

```cpp
struct TDimensions {
    float left;     // mm - límite izquierdo
    float top;      // mm - límite superior
    float width;    // mm - ancho del grid
    float height;   // mm - alto del grid
};
```

### Result (struct)

Resultado de la computación de paths.

```cpp
struct Result {
    TPaths paths;       // Paths computados (puede estar vacío)
    long timestamp;     // Timestamp de computación (ms desde epoch)
    string errorMsg;    // Mensaje de error/estado
    bool valid;         // True si la computación tuvo éxito
};
```

### TCell (struct)

Representación compacta de celda para transmisión.

```cpp
struct TCell {
    int x;          // mm - posición X de la celda (centro)
    int y;          // mm - posición Y de la celda (centro)
    byte cost;      // 0=libre, 255=obstáculo, 100-200=capas de inflación
};
```

### Map (struct)

Mapa serializado para transmisión por red.

```cpp
struct Map {
    int tileSize;           // mm - tamaño de cada celda (típicamente 100mm)
    TCellVector cells;      // Celdas con cost > 0
};
```

### Pose (struct)

Pose del robot con covarianza.

```cpp
struct Pose {
    float x;          // mm - posición X
    float y;          // mm - posición Y
    float theta;      // radianes - orientación
    Covariance cov;   // Matriz de covarianza 3x3 aplanada [xx, xy, xθ, yy, yθ, θθ]
};
```

### NavigationState (enum)

Estado actual de la navegación.

| Valor | Descripción |
|-------|-------------|
| `IDLE` | Sin target, robot parado |
| `NAVIGATING` | En movimiento hacia el target |
| `PAUSED` | Detenido pero con target activo (puede reanudar) |
| `REACHED` | Target alcanzado, robot parado |
| `BLOCKED` | Path bloqueado, intentando replanificar |
| `ERROR` | Error crítico (localización perdida, etc.) |

### NavigationOptions (struct)

Opciones para personalizar el comportamiento de navegación.

```cpp
struct NavigationOptions {
    float maxSpeed;         // mm/s - velocidad máxima permitida (0 = usar default)
    float safetyFactor;     // 0.0-1.0 - preferir paths más seguros (0=rápido, 1=seguro)
    bool useEsdf;           // Usar ESDF para costes suaves de obstáculos
    bool allowReplan;       // Permitir replanificación automática si el path está bloqueado
};
```

### NavigationStatus (struct)

Estado completo de la navegación.

```cpp
struct NavigationStatus {
    NavigationState state;          // Estado actual de navegación
    TPoint currentTarget;           // Target actual (0,0 si IDLE)
    TPoint currentPosition;         // Posición estimada del robot
    float currentOrientation;       // Orientación estimada (radianes)
    float distanceToTarget;         // mm - distancia restante al target
    float estimatedTime;            // segundos - tiempo estimado de llegada
    float currentSpeed;             // mm/s - velocidad actual del robot
    int pathWaypointsRemaining;     // Waypoints restantes en el path
    string statusMessage;           // Mensaje de estado legible
};
```

---

## Path Planning

### getPaths

Computa múltiples paths desde un origen a un destino.

```cpp
Result getPaths(TPoint source, TPoint target, int maxPaths,
                bool tryClosestFreePoint, bool targetIsHuman, float safetyFactor)
```

**Parámetros:**
- `source`: Punto de origen
- `target`: Punto destino
- `maxPaths`: Número máximo de paths alternativos a computar
- `tryClosestFreePoint`: Si `true`, busca el punto libre más cercano si el target está bloqueado
- `targetIsHuman`: Si `true`, aplica planificación consciente de humanos
- `safetyFactor`: 0.0=path más corto (roza paredes), 1.0=path más seguro (prefiere centro)

**Retorna:**
- `Result` con los paths computados y estado

**Ejemplo:**
```cpp
RoboCompGridder::TPoint origen{0.f, 0.f, 0.f};
RoboCompGridder::TPoint destino{5000.f, 3000.f, 0.f};

auto result = gridder_proxy->getPaths(origen, destino, 3, true, false, 0.7f);

if (result.valid && !result.paths.empty()) {
    auto mejor_path = result.paths[0];
    std::cout << "Path encontrado con " << mejor_path.size() << " waypoints" << std::endl;
} else {
    std::cerr << "Error: " << result.errorMsg << std::endl;
}
```

---

### getClosestFreePoint

Encuentra el punto libre más cercano a un punto dado.

```cpp
TPoint getClosestFreePoint(TPoint source)
```

**Parámetros:**
- `source`: Punto desde el cual buscar

**Retorna:**
- Punto libre más cercano, o `(0, 0, 0)` si no se encuentra

**Ejemplo:**
```cpp
RoboCompGridder::TPoint punto_bloqueado{1000.f, 1000.f, 0.f};
auto punto_libre = gridder_proxy->getClosestFreePoint(punto_bloqueado);

if (punto_libre.x != 0.f || punto_libre.y != 0.f) {
    std::cout << "Punto libre encontrado en: " << punto_libre.x << ", " << punto_libre.y << std::endl;
}
```

---

### LineOfSightToTarget

Verifica si hay línea de visión directa entre dos puntos.

```cpp
bool LineOfSightToTarget(TPoint source, TPoint target, float robotRadius)
```

**Parámetros:**
- `source`: Punto de origen
- `target`: Punto destino
- `robotRadius`: Radio del robot para verificación de colisiones (mm)

**Retorna:**
- `true` si la línea de visión está libre
- `false` si hay obstáculos

**Ejemplo:**
```cpp
RoboCompGridder::TPoint robot{0.f, 0.f, 0.f};
RoboCompGridder::TPoint objetivo{3000.f, 2000.f, 0.f};
float radio_robot = 230.f;  // mm

if (gridder_proxy->LineOfSightToTarget(robot, objetivo, radio_robot)) {
    std::cout << "Camino directo disponible!" << std::endl;
} else {
    std::cout << "Necesita planificación de path" << std::endl;
}
```

---

### IsPathBlocked

Verifica si un path está bloqueado por obstáculos.

```cpp
bool IsPathBlocked(TPath path)
```

**Parámetros:**
- `path`: Path a verificar (secuencia de waypoints)

**Retorna:**
- `true` si el path está bloqueado
- `false` si el path está libre

**Ejemplo:**
```cpp
RoboCompGridder::TPath mi_path;
// ... llenar mi_path con waypoints ...

if (gridder_proxy->IsPathBlocked(mi_path)) {
    std::cout << "Path bloqueado, replanificando..." << std::endl;
    gridder_proxy->replanPath();
}
```

---

### setLocationAndGetPath

Actualiza el grid con puntos libres/obstáculos y computa un path.

```cpp
Result setLocationAndGetPath(TPoint source, TPoint target,
                             TPointVector freePoints, TPointVector obstaclePoints)
```

**Parámetros:**
- `source`: Punto de origen
- `target`: Punto destino
- `freePoints`: Puntos conocidos como libres
- `obstaclePoints`: Puntos conocidos como obstáculos

**Retorna:**
- `Result` con el path computado

**Ejemplo:**
```cpp
RoboCompGridder::TPoint origen{0.f, 0.f, 0.f};
RoboCompGridder::TPoint destino{5000.f, 3000.f, 0.f};

RoboCompGridder::TPointVector libres;
RoboCompGridder::TPointVector obstaculos;

// Añadir puntos del LiDAR como obstáculos
for (const auto& pt : lidar_points) {
    obstaculos.push_back({pt.x, pt.y, 0.f});
}

auto result = gridder_proxy->setLocationAndGetPath(origen, destino, libres, obstaculos);
```

---

## Grid y Mapa

### getDimensions

Obtiene las dimensiones actuales del grid.

```cpp
TDimensions getDimensions()
```

**Retorna:**
- Estructura `TDimensions` con los límites del grid

**Ejemplo:**
```cpp
auto dims = gridder_proxy->getDimensions();
std::cout << "Grid: " << dims.width << "x" << dims.height << " mm" << std::endl;
std::cout << "Origen: (" << dims.left << ", " << dims.top << ")" << std::endl;
```

---

### setGridDimensions

Establece las dimensiones del grid (borra el grid existente).

```cpp
bool setGridDimensions(TDimensions dimensions)
```

**Parámetros:**
- `dimensions`: Nuevos límites del grid

**Retorna:**
- `true` si se estableció correctamente

**Ejemplo:**
```cpp
RoboCompGridder::TDimensions dims;
dims.left = -10000.f;   // -10 metros
dims.top = -10000.f;    // -10 metros  
dims.width = 20000.f;   // 20 metros
dims.height = 20000.f;  // 20 metros

if (gridder_proxy->setGridDimensions(dims)) {
    std::cout << "Grid redimensionado" << std::endl;
}
```

---

### getMap

Obtiene el mapa serializado (solo celdas con coste > 0).

```cpp
Map getMap()
```

**Retorna:**
- Estructura `Map` con representación compacta del mapa

**Ejemplo:**
```cpp
auto mapa = gridder_proxy->getMap();

std::cout << "Tamaño de celda: " << mapa.tileSize << " mm" << std::endl;
std::cout << "Celdas con obstáculos: " << mapa.cells.size() << std::endl;

// Procesar celdas
for (const auto& celda : mapa.cells) {
    if (celda.cost == 255) {
        std::cout << "Obstáculo en: (" << celda.x << ", " << celda.y << ")" << std::endl;
    }
}
```

---

## Localización

### getPose

Obtiene la pose actual del robot con covarianza.

```cpp
Pose getPose()
```

**Retorna:**
- Estructura `Pose` con posición, orientación y covarianza

**Ejemplo:**
```cpp
auto pose = gridder_proxy->getPose();

std::cout << "Posición: (" << pose.x << ", " << pose.y << ") mm" << std::endl;
std::cout << "Orientación: " << pose.theta * 180.f / M_PI << "°" << std::endl;

// Covarianza (matriz 3x3 aplanada: [xx, xy, xθ, yy, yθ, θθ])
if (!pose.cov.empty()) {
    float sigma_x = std::sqrt(pose.cov[0]);  // Desviación estándar en X
    float sigma_y = std::sqrt(pose.cov[3]);  // Desviación estándar en Y
    float sigma_theta = std::sqrt(pose.cov[5]);  // Desviación estándar en θ
    
    std::cout << "Incertidumbre: σx=" << sigma_x << "mm, σy=" << sigma_y 
              << "mm, σθ=" << sigma_theta << "rad" << std::endl;
}
```

---

## Gestión de Target

### setTarget

Establece un destino y comienza la navegación automáticamente.

```cpp
bool setTarget(TPoint target)
```

**Parámetros:**
- `target`: Punto destino en coordenadas del mapa (mm)

**Retorna:**
- `true` si el target es válido y se encontró un path
- `false` si el target está bloqueado o no hay path

**Ejemplo:**
```cpp
RoboCompGridder::TPoint destino;
destino.x = 5000.f;  // 5 metros en X
destino.y = 3000.f;  // 3 metros en Y
destino.radius = 0.f;

if (gridder_proxy->setTarget(destino)) {
    std::cout << "Navegación iniciada" << std::endl;
} else {
    std::cout << "No se puede alcanzar el destino" << std::endl;
}
```

---

### setTargetWithOptions

Establece un destino con opciones personalizadas.

```cpp
bool setTargetWithOptions(TPoint target, NavigationOptions options)
```

**Parámetros:**
- `target`: Punto destino
- `options`: Opciones de navegación

**Ejemplo:**
```cpp
RoboCompGridder::TPoint destino{5000.f, 3000.f, 0.f};

RoboCompGridder::NavigationOptions opts;
opts.maxSpeed = 300.f;      // Máximo 300 mm/s
opts.safetyFactor = 0.8f;   // Preferir paths seguros
opts.useEsdf = true;
opts.allowReplan = true;

gridder_proxy->setTargetWithOptions(destino, opts);
```

---

## Control de Ejecución

### startNavigation

Inicia o reanuda la navegación hacia el target actual.

```cpp
bool startNavigation()
```

**Retorna:**
- `true` si la navegación se inició correctamente
- `false` si no hay target establecido

---

### stopNavigation

Detiene el robot pero **mantiene el target actual**. El robot puede reanudar con `resumeNavigation()`.

```cpp
void stopNavigation()
```

**Comportamiento:**
- Envía comando de velocidad cero al robot
- Cambia estado a `IDLE` (equivalente a PAUSED)
- El target y el path se mantienen

---

### resumeNavigation

Reanuda la navegación después de una pausa/stop.

```cpp
bool resumeNavigation()
```

**Retorna:**
- `true` si se reanudó correctamente
- `false` si no hay target o ya está navegando

---

### cancelNavigation

Cancela la navegación completamente. Detiene el robot **Y borra el target**.

```cpp
void cancelNavigation()
```

**Comportamiento:**
- Envía comando de velocidad cero
- Borra el target y el path
- Cambia estado a `IDLE`
- Resetea el controlador MPPI

---

### replanPath

Fuerza una replanificación del path hacia el target actual. Útil cuando el entorno ha cambiado.

```cpp
bool replanPath()
```

**Retorna:**
- `true` si se encontró un nuevo path
- `false` si no hay target o no se encontró path

---

## Consulta de Estado

### getNavigationState

Obtiene el estado actual de navegación.

```cpp
NavigationState getNavigationState()
```

**Ejemplo:**
```cpp
auto state = gridder_proxy->getNavigationState();
switch (state) {
    case RoboCompGridder::NavigationState::IDLE:
        std::cout << "Robot parado, sin target" << std::endl;
        break;
    case RoboCompGridder::NavigationState::NAVIGATING:
        std::cout << "Navegando..." << std::endl;
        break;
    case RoboCompGridder::NavigationState::REACHED:
        std::cout << "¡Target alcanzado!" << std::endl;
        break;
    case RoboCompGridder::NavigationState::BLOCKED:
        std::cout << "Path bloqueado" << std::endl;
        break;
}
```

---

### getNavigationStatus

Obtiene el estado completo de navegación en una sola llamada.

```cpp
NavigationStatus getNavigationStatus()
```

**Ejemplo:**
```cpp
auto status = gridder_proxy->getNavigationStatus();

std::cout << "Estado: " << status.statusMessage << std::endl;
std::cout << "Posición: (" << status.currentPosition.x << ", " 
          << status.currentPosition.y << ")" << std::endl;
std::cout << "Distancia al target: " << status.distanceToTarget << " mm" << std::endl;
std::cout << "Tiempo estimado: " << status.estimatedTime << " s" << std::endl;
std::cout << "Velocidad actual: " << status.currentSpeed << " mm/s" << std::endl;
```

---

### getTarget

Obtiene el target actual.

```cpp
TPoint getTarget()
```

**Retorna:**
- Target actual, o `(0, 0, 0)` si no hay target

---

### getDistanceToTarget

Obtiene la distancia euclidiana al target.

```cpp
float getDistanceToTarget()
```

**Retorna:**
- Distancia en mm
- `0` si no hay target o ya se alcanzó
- `-1` si no se puede determinar (sin pose)

---

### getEstimatedTimeToTarget

Obtiene el tiempo estimado de llegada.

```cpp
float getEstimatedTimeToTarget()
```

**Retorna:**
- Tiempo en segundos
- `-1` si no se puede estimar

---

### hasReachedTarget

Verifica si el robot ha alcanzado el target.

```cpp
bool hasReachedTarget()
```

**Retorna:**
- `true` si `state == REACHED`
- `false` en cualquier otro caso

---

## Diagrama de Estados

```
                     setTarget()
        ┌─────────────────────────────────────┐
        │                                     ▼
      IDLE ◄────────── cancelNavigation() ── NAVIGATING
        ▲                                     │ │
        │                                     │ │ stopNavigation()
        │  (timeout)                          │ ▼
      ERROR ◄─────────────────────────────  PAUSED/IDLE
        ▲                                     │
        │                                     │ resumeNavigation()
        │                                     ▼
        │              goal reached      NAVIGATING
        │                    │
        │                    ▼
        └──────────────── REACHED
              (auto después de nuevo setTarget)
```

### Transiciones de Estado

| Desde | Acción | Hacia |
|-------|--------|-------|
| `IDLE` | `setTarget()` | `NAVIGATING` o `BLOCKED` |
| `NAVIGATING` | llegada a meta | `REACHED` |
| `NAVIGATING` | `stopNavigation()` | `IDLE` (paused) |
| `NAVIGATING` | `cancelNavigation()` | `IDLE` |
| `NAVIGATING` | path bloqueado | `BLOCKED` |
| `IDLE` (paused) | `resumeNavigation()` | `NAVIGATING` |
| `BLOCKED` | `replanPath()` exitoso | `NAVIGATING` |
| `REACHED` | `setTarget()` nuevo | `NAVIGATING` |

---

## Ejemplos de Uso

### Ejemplo 1: Navegación Simple

```cpp
#include <Gridder.h>

void navegar_a_punto(RoboCompGridder::GridderPrxPtr gridder, float x, float y)
{
    // Establecer destino
    RoboCompGridder::TPoint destino{x, y, 0.f};
    
    if (!gridder->setTarget(destino)) {
        std::cerr << "No se puede navegar al punto" << std::endl;
        return;
    }
    
    // Esperar a que llegue
    while (!gridder->hasReachedTarget()) {
        auto status = gridder->getNavigationStatus();
        std::cout << "Distancia: " << status.distanceToTarget << " mm" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    std::cout << "¡Llegamos!" << std::endl;
}
```

### Ejemplo 2: Navegación con Manejo de Errores

```cpp
void navegar_seguro(RoboCompGridder::GridderPrxPtr gridder, float x, float y)
{
    RoboCompGridder::TPoint destino{x, y, 0.f};
    
    if (!gridder->setTarget(destino)) {
        std::cerr << "Target inválido o bloqueado" << std::endl;
        return;
    }
    
    while (true) {
        auto state = gridder->getNavigationState();
        
        switch (state) {
            case RoboCompGridder::NavigationState::REACHED:
                std::cout << "Objetivo alcanzado" << std::endl;
                return;
                
            case RoboCompGridder::NavigationState::BLOCKED:
                std::cout << "Path bloqueado, replanificando..." << std::endl;
                if (!gridder->replanPath()) {
                    std::cerr << "No se pudo encontrar path alternativo" << std::endl;
                    gridder->cancelNavigation();
                    return;
                }
                break;
                
            case RoboCompGridder::NavigationState::ERROR:
                std::cerr << "Error de navegación" << std::endl;
                gridder->cancelNavigation();
                return;
                
            default:
                // NAVIGATING o IDLE - continuar esperando
                break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
```

### Ejemplo 3: Verificación de Path Antes de Navegar

```cpp
void navegar_con_verificacion(RoboCompGridder::GridderPrxPtr gridder, 
                               float x, float y, float robot_radius)
{
    auto pose = gridder->getPose();
    RoboCompGridder::TPoint origen{pose.x, pose.y, 0.f};
    RoboCompGridder::TPoint destino{x, y, 0.f};
    
    // Verificar si hay línea de visión directa
    if (gridder->LineOfSightToTarget(origen, destino, robot_radius)) {
        std::cout << "Camino directo disponible" << std::endl;
    } else {
        // Computar path
        auto result = gridder->getPaths(origen, destino, 1, true, false, 0.5f);
        if (!result.valid || result.paths.empty()) {
            std::cerr << "No se encontró path: " << result.errorMsg << std::endl;
            return;
        }
        std::cout << "Path encontrado con " << result.paths[0].size() << " waypoints" << std::endl;
    }
    
    // Navegar
    gridder->setTarget(destino);
}
```

### Ejemplo 4: Pausa y Reanudación

```cpp
void navegar_con_pausa(RoboCompGridder::GridderPrxPtr gridder)
{
    // Iniciar navegación
    gridder->setTarget({5000.f, 3000.f, 0.f});
    
    // Esperar un poco
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Pausar (persona detectada, por ejemplo)
    std::cout << "Pausando navegación..." << std::endl;
    gridder->stopNavigation();
    
    // Esperar a que la persona pase
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // Reanudar
    std::cout << "Reanudando..." << std::endl;
    gridder->resumeNavigation();
}
```

### Ejemplo 5: Monitorización Continua

```cpp
void monitor_navegacion(RoboCompGridder::GridderPrxPtr gridder)
{
    while (true) {
        auto status = gridder->getNavigationStatus();
        
        std::cout << "\033[2J\033[H";  // Limpiar pantalla
        std::cout << "=== Estado de Navegación ===" << std::endl;
        std::cout << "Estado: " << status.statusMessage << std::endl;
        std::cout << "Posición: (" << status.currentPosition.x << ", " 
                  << status.currentPosition.y << ")" << std::endl;
        std::cout << "Orientación: " << status.currentOrientation * 180.f / M_PI 
                  << "°" << std::endl;
        std::cout << "Target: (" << status.currentTarget.x << ", " 
                  << status.currentTarget.y << ")" << std::endl;
        std::cout << "Distancia: " << status.distanceToTarget / 1000.f 
                  << " m" << std::endl;
        std::cout << "ETA: " << status.estimatedTime << " s" << std::endl;
        std::cout << "Velocidad: " << status.currentSpeed << " mm/s" << std::endl;
        std::cout << "Waypoints: " << status.pathWaypointsRemaining << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}
```

### Ejemplo 6: Obtener y Visualizar Mapa

```cpp
void mostrar_info_mapa(RoboCompGridder::GridderPrxPtr gridder)
{
    auto dims = gridder->getDimensions();
    auto mapa = gridder->getMap();
    
    std::cout << "=== Información del Mapa ===" << std::endl;
    std::cout << "Dimensiones: " << dims.width / 1000.f << " x " 
              << dims.height / 1000.f << " metros" << std::endl;
    std::cout << "Origen: (" << dims.left / 1000.f << ", " 
              << dims.top / 1000.f << ") metros" << std::endl;
    std::cout << "Tamaño de celda: " << mapa.tileSize << " mm" << std::endl;
    std::cout << "Celdas ocupadas: " << mapa.cells.size() << std::endl;
    
    // Contar por tipo
    int obstaculos = 0, inflacion = 0;
    for (const auto& c : mapa.cells) {
        if (c.cost == 255) obstaculos++;
        else if (c.cost >= 100) inflacion++;
    }
    std::cout << "  - Obstáculos: " << obstaculos << std::endl;
    std::cout << "  - Inflación: " << inflacion << std::endl;
}
```

---

## Resumen de Métodos

### Path Planning
| Método | Descripción |
|--------|-------------|
| `getPaths()` | Computa múltiples paths alternativos |
| `getClosestFreePoint()` | Encuentra punto libre más cercano |
| `LineOfSightToTarget()` | Verifica línea de visión directa |
| `IsPathBlocked()` | Verifica si un path está bloqueado |
| `setLocationAndGetPath()` | Actualiza grid y computa path |

### Grid y Mapa
| Método | Descripción |
|--------|-------------|
| `getDimensions()` | Obtiene dimensiones del grid |
| `setGridDimensions()` | Establece dimensiones del grid |
| `getMap()` | Obtiene mapa serializado |

### Localización
| Método | Descripción |
|--------|-------------|
| `getPose()` | Obtiene pose del robot con covarianza |

### Navegación
| Método | Descripción |
|--------|-------------|
| `setTarget()` | Establece destino e inicia navegación |
| `setTargetWithOptions()` | Establece destino con opciones |
| `startNavigation()` | Inicia/reanuda navegación |
| `stopNavigation()` | Pausa navegación (mantiene target) |
| `resumeNavigation()` | Reanuda navegación pausada |
| `cancelNavigation()` | Cancela y borra target |
| `replanPath()` | Fuerza replanificación |

### Consulta de Estado
| Método | Descripción |
|--------|-------------|
| `getNavigationState()` | Estado de navegación (enum) |
| `getNavigationStatus()` | Estado completo (struct) |
| `getTarget()` | Target actual |
| `getDistanceToTarget()` | Distancia restante (mm) |
| `getEstimatedTimeToTarget()` | Tiempo estimado (s) |
| `hasReachedTarget()` | Verifica si llegó al target |

---

## Notas Importantes

1. **Coordenadas**: Todas las coordenadas están en milímetros (mm)
2. **Ángulos**: Orientaciones en radianes
3. **Thread-Safety**: Todos los métodos son thread-safe
4. **Timeout**: No hay timeout automático - el componente navega hasta alcanzar el target o ser cancelado
5. **Replanificación**: Se recomienda habilitar `allowReplan` en `NavigationOptions` para entornos dinámicos
6. **Covarianza**: La matriz de covarianza es simétrica 3x3 aplanada: `[σxx, σxy, σxθ, σyy, σyθ, σθθ]`

---

## Véase También

- [GRIDDER_DOCUMENTATION.md](GRIDDER_DOCUMENTATION.md) - Documentación técnica completa
- [USER_MANUAL.md](USER_MANUAL.md) - Manual de usuario
- [README.md](README.md) - Descripción general del componente


