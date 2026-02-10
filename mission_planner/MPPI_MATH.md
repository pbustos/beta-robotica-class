# MPPI - Model Predictive Path Integral Control

## Descripción General

MPPI (Model Predictive Path Integral) es un algoritmo de control óptimo estocástico basado en muestreo. A diferencia de métodos tradicionales como MPC (Model Predictive Control), MPPI no requiere resolver un problema de optimización no lineal, sino que aproxima la solución óptima mediante un promedio ponderado de trayectorias muestreadas.

## Fundamentos Matemáticos

### 1. Dinámica del Sistema

El sistema se modela con la siguiente ecuación en tiempo discreto:

$$\mathbf{x}_{t+1} = f(\mathbf{x}_t, \mathbf{u}_t)$$

donde:
- $\mathbf{x}_t \in \mathbb{R}^n$ : estado del sistema en tiempo $t$
- $\mathbf{u}_t \in \mathbb{R}^m$ : entrada de control en tiempo $t$
- $f(\cdot)$ : función de transición de estados

**Para nuestro robot omnidireccional:**

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}, \quad \mathbf{u} = \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}$$

El modelo cinemático es:

$$\begin{aligned}
x_{t+1} &= x_t + (v_x \cos\theta_t - v_y \sin\theta_t) \cdot \Delta t \\
y_{t+1} &= y_t + (v_x \sin\theta_t + v_y \cos\theta_t) \cdot \Delta t \\
\theta_{t+1} &= \theta_t + \omega \cdot \Delta t
\end{aligned}$$

### 2. Función de Costo

El objetivo es minimizar el costo esperado sobre un horizonte $T$:

$$J(\mathbf{U}) = \mathbb{E}\left[ \phi(\mathbf{x}_T) + \sum_{t=0}^{T-1} q(\mathbf{x}_t, \mathbf{u}_t) \right]$$

donde:
- $\mathbf{U} = [\mathbf{u}_0, \mathbf{u}_1, ..., \mathbf{u}_{T-1}]$ : secuencia de controles
- $\phi(\mathbf{x}_T)$ : costo terminal
- $q(\mathbf{x}_t, \mathbf{u}_t)$ : costo instantáneo (running cost)

**Nuestra función de costo incluye:**

$$q(\mathbf{x}_t, \mathbf{u}_t) = w_p \cdot c_{path} + w_o \cdot c_{obs} + w_g \cdot c_{goal} + w_s \cdot c_{smooth} + w_v \cdot c_{speed}$$

| Término | Fórmula | Descripción |
|---------|---------|-------------|
| $c_{path}$ | $d_{path}^2$ | Distancia al camino al cuadrado |
| $c_{obs}$ | $\exp\left(\frac{d_{margin} - d_{obs}}{d_{decay}}\right)$ | Penalización exponencial por proximidad a obstáculos |
| $c_{goal}$ | $\|\mathbf{x} - \mathbf{x}_{goal}\|$ | Distancia al objetivo |
| $c_{smooth}$ | $\|\mathbf{u}_t - \mathbf{u}_{t-1}\|^2$ | Cambio en controles |
| $c_{speed}$ | $(v - v_{desired})^2$ | Desviación de velocidad deseada |

### 3. Muestreo de Trayectorias

MPPI genera $K$ secuencias de control perturbadas:

$$\mathbf{u}_t^{(k)} = \bar{\mathbf{u}}_t + \boldsymbol{\epsilon}_t^{(k)}$$

donde:
- $\bar{\mathbf{u}}_t$ : control nominal (de la iteración anterior)
- $\boldsymbol{\epsilon}_t^{(k)} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma})$ : ruido gaussiano

**Matriz de covarianza del ruido:**

$$\boldsymbol{\Sigma} = \begin{bmatrix} \sigma_{v_x}^2 & 0 & 0 \\ 0 & \sigma_{v_y}^2 & 0 \\ 0 & 0 & \sigma_{\omega}^2 \end{bmatrix}$$

### 4. Evaluación de Trayectorias

Para cada muestra $k$, se simula la trayectoria completa:

$$\mathbf{x}_{t+1}^{(k)} = f(\mathbf{x}_t^{(k)}, \mathbf{u}_t^{(k)})$$

Y se calcula el costo total:

$$S^{(k)} = \phi(\mathbf{x}_T^{(k)}) + \sum_{t=0}^{T-1} q(\mathbf{x}_t^{(k)}, \mathbf{u}_t^{(k)})$$

### 5. Cálculo de Pesos

Los pesos se calculan usando la función **softmin** (distribución de Boltzmann):

$$w^{(k)} = \frac{\exp\left(-\frac{1}{\lambda}(S^{(k)} - S_{min})\right)}{\sum_{j=1}^{K} \exp\left(-\frac{1}{\lambda}(S^{(j)} - S_{min})\right)}$$

donde:
- $\lambda > 0$ : parámetro de **temperatura**
- $S_{min} = \min_k S^{(k)}$ : costo mínimo (para estabilidad numérica)

**Efecto de $\lambda$:**
- $\lambda \to 0$ : comportamiento **greedy** (solo la mejor trayectoria)
- $\lambda \to \infty$ : promedio uniforme de todas las trayectorias

### 6. Actualización del Control

El control óptimo se obtiene como media ponderada:

$$\mathbf{u}_t^* = \sum_{k=1}^{K} w^{(k)} \cdot \mathbf{u}_t^{(k)}$$

Equivalentemente:

$$\mathbf{u}_t^* = \bar{\mathbf{u}}_t + \sum_{k=1}^{K} w^{(k)} \cdot \boldsymbol{\epsilon}_t^{(k)}$$

### 7. Warm Starting

Para mejorar la convergencia, la secuencia de control se desplaza en cada iteración:

$$\bar{\mathbf{u}}_t^{new} = \mathbf{u}_{t+1}^{*,old} \quad \text{para } t = 0, ..., T-2$$

$$\bar{\mathbf{u}}_{T-1}^{new} = \mathbf{u}_{T-1}^{*,old}$$

---

## Algoritmo Completo

```
Algoritmo: MPPI Controller
─────────────────────────────────────────────────────────
Entrada: estado actual x₀, path objetivo, obstáculos
Salida: comando de control óptimo u*

1. Warm start: desplazar secuencia de control anterior
   ū ← shift(ū_prev)

2. Para k = 1 hasta K:
   a. Generar secuencia de ruido:
      ε^(k) ← sample(N(0, Σ), T)
   
   b. Calcular secuencia de control perturbada:
      u^(k) ← clamp(ū + ε^(k), u_min, u_max)
   
   c. Simular trayectoria:
      x^(k)_0 ← x₀
      Para t = 0 hasta T-1:
         x^(k)_{t+1} ← f(x^(k)_t, u^(k)_t)
   
   d. Calcular costo total:
      S^(k) ← Σ_t q(x^(k)_t, u^(k)_t)

3. Encontrar costo mínimo:
   S_min ← min_k(S^(k))

4. Calcular pesos normalizados:
   Para k = 1 hasta K:
      w^(k) ← exp(-1/λ · (S^(k) - S_min))
   w ← w / sum(w)

5. Calcular control óptimo:
   u* ← Σ_k w^(k) · u^(k)

6. Guardar para siguiente iteración:
   ū_prev ← u*

7. Retornar u*_0 (primer control de la secuencia)
─────────────────────────────────────────────────────────
```

---

## Diagrama de Flujo

```
                    ┌─────────────────┐
                    │  Estado actual  │
                    │   x₀, θ₀        │
                    └────────┬────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │      Warm Start              │
              │  ū ← shift(ū_prev)           │
              └──────────────┬───────────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ▼                   ▼                   ▼
    ┌─────────┐         ┌─────────┐         ┌─────────┐
    │Sample 1 │         │Sample 2 │   ...   │Sample K │
    │ε¹~N(0,Σ)│         │ε²~N(0,Σ)│         │εᴷ~N(0,Σ)│
    └────┬────┘         └────┬────┘         └────┬────┘
         │                   │                   │
         ▼                   ▼                   ▼
    ┌─────────┐         ┌─────────┐         ┌─────────┐
    │Simulate │         │Simulate │   ...   │Simulate │
    │Trajectory│        │Trajectory│        │Trajectory│
    └────┬────┘         └────┬────┘         └────┬────┘
         │                   │                   │
         ▼                   ▼                   ▼
    ┌─────────┐         ┌─────────┐         ┌─────────┐
    │ Cost S¹ │         │ Cost S² │   ...   │ Cost Sᴷ │
    └────┬────┘         └────┬────┘         └────┬────┘
         │                   │                   │
         └───────────────────┼───────────────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │     Compute Weights          │
              │  wᵏ = exp(-Sᵏ/λ) / Σexp()   │
              └──────────────┬───────────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │   Weighted Average           │
              │   u* = Σ wᵏ · uᵏ            │
              └──────────────┬───────────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │   Apply u*₀ to robot         │
              │   (vx, vy, ω)                │
              └──────────────────────────────┘
```

---

## Parámetros de Configuración

| Parámetro | Símbolo | Valor Default | Descripción |
|-----------|---------|---------------|-------------|
| Número de muestras | $K$ | 1000 | Más muestras = mejor aproximación |
| Horizonte | $T$ | 30 | Pasos de predicción |
| Tiempo de paso | $\Delta t$ | 0.05 s | Discretización temporal |
| Temperatura | $\lambda$ | 1.0 | Exploración vs explotación |
| Ruido $v_x$ | $\sigma_{v_x}$ | 200 mm/s | Desviación estándar |
| Ruido $v_y$ | $\sigma_{v_y}$ | 200 mm/s | Desviación estándar |
| Ruido $\omega$ | $\sigma_\omega$ | 0.3 rad/s | Desviación estándar |
| Radio robot | $r$ | 250 mm | Para detección de colisión |
| Margen seguridad | $d_{margin}$ | 400 mm | Inicio de penalización |

---

## Complejidad Computacional

- **Tiempo**: $O(K \cdot T \cdot (n + m + |obstacles|))$
- **Espacio**: $O(K \cdot T \cdot m)$ para almacenar secuencias de control

Donde:
- $K$ = número de muestras
- $T$ = horizonte de predicción
- $n$ = dimensión del estado
- $m$ = dimensión del control
- $|obstacles|$ = número de puntos de obstáculos

---

## Ventajas de MPPI

1. **Libre de derivadas**: No requiere gradientes de la función de costo
2. **Maneja costos no suaves**: Puede usar funciones de costo discontinuas
3. **Paralelizable**: Las K trayectorias se pueden evaluar en paralelo (GPU)
4. **Robusto**: Explora múltiples soluciones simultáneamente
5. **Fácil de implementar**: No requiere solvers de optimización complejos

## Limitaciones

1. **Dependencia de muestras**: Necesita muchas muestras para buena aproximación
2. **Horizonte limitado**: No garantiza optimalidad global
3. **Costo computacional**: Puede ser costoso para sistemas de alta dimensión

---

## Referencias

1. Williams, G., et al. "Information theoretic MPC for model-based reinforcement learning." ICRA 2017.
2. Williams, G., et al. "Aggressive driving with model predictive path integral control." ICRA 2016.
3. Theodorou, E., et al. "A generalized path integral control approach to reinforcement learning." JMLR 2010.
