# mcl_puzzlebot

Monte Carlo Localization (MCL) para el robot Puzzlebot en Gazebo Classic + ROS 2 Humble.

Implementa la **Actividad 5** de la presentación de Semana 3 (pasos A–I): localización 2D con filtro de partículas usando un mapa conocido y datos de LIDAR.

---

## Contenido del paquete

```
mcl_puzzlebot/
├── mcl_puzzlebot/
│   └── mcl_node.py          # Nodo ROS 2: filtro de partículas MCL
├── urdf/
│   └── puzzlebot_gazebo.urdf  # URDF con colisiones, inercias y plugins Gazebo
├── worlds/
│   └── mcl_room.world       # Cuarto SDF 5×5 m con 4 paredes
├── maps/
│   ├── room_map.png          # Mapa PNG 600×600 px (negro=pared, blanco=libre)
│   └── room_map.yaml         # Metadatos del mapa (resolución, origen)
├── launch/
│   └── mcl_launch.py        # Lanza Gazebo + robot + MCL + RViz
└── rviz/
    └── mcl_rviz.rviz         # Config de RViz (partículas, laser, pose)
```

---

## Cómo correrlo

### Prerequisitos

```bash
# ROS 2 Humble + Gazebo Classic 11
sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-teleop-twist-keyboard

# Workspace con puzzlebot_sim (para los STL del robot)
cd ~/ros2_ws && colcon build --packages-select puzzlebot_sim mcl_puzzlebot
```

### Lanzar la simulación

```bash
# Terminal 1 — Gazebo + robot + MCL + RViz (todo junto)
source ~/ros2_ws/install/setup.bash
ros2 launch mcl_puzzlebot mcl_launch.py
```

Espera ~15 s a que Gazebo termine de cargar. Verás:
- **Gazebo**: cuarto blanco con 4 paredes grises y el Puzzlebot en el centro.
- **RViz**: puntos rojos del laser, flechas verdes (partículas), flecha roja grande (pose estimada).

### Mover el robot

```bash
# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| Tecla | Acción |
|-------|--------|
| `i`   | Avanzar |
| `,`   | Retroceder |
| `j`   | Girar izquierda |
| `l`   | Girar derecha |
| `k`   | Frenar |
| `u/o` | Avanzar diagonal |

Conforme el robot se mueva, las partículas verdes convergen hacia la posición real.

### Tópicos relevantes

| Tópico | Tipo | Descripción |
|--------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Lecturas del LIDAR (Gazebo → MCL) |
| `/odom` | `nav_msgs/Odometry` | Odometría del diff-drive (Gazebo → MCL) |
| `/cmd_vel` | `geometry_msgs/Twist` | Comandos de velocidad (teleop → Gazebo) |
| `/particle_cloud` | `geometry_msgs/PoseArray` | Las N partículas (MCL → RViz) |
| `/mcl_pose` | `geometry_msgs/PoseStamped` | Pose estimada — mejor partícula (MCL → RViz) |

---

## Cómo funciona — Matemáticas

### El problema de localización

El robot conoce el mapa del entorno pero **no sabe dónde está**. El objetivo es estimar la pose $\mathbf{x}_t = (x, y, \theta)$ en cada instante $t$ a partir de:
- Las mediciones del sensor: $\mathbf{z}_t$ (scan del LIDAR)
- Los comandos de movimiento: $\mathbf{u}_t$ (odometría)

La solución bayesiana es mantener la **distribución de creencia**:

$$\text{bel}(\mathbf{x}_t) = p(\mathbf{x}_t \mid \mathbf{z}_{1:t},\, \mathbf{u}_{1:t})$$

MCL aproxima esta distribución con un conjunto de $N$ partículas $\{\mathbf{x}_t^{[i]}\}_{i=1}^N$.

---

### Paso D — Muestreo inicial de partículas

Las $N$ partículas se distribuyen **uniformemente** sobre el espacio libre del mapa:

$$\mathbf{x}^{[i]} = (x^{[i]},\, y^{[i]},\, \theta^{[i]}), \quad i = 1, \ldots, N$$

donde $(x^{[i]}, y^{[i]})$ se samplea de los pixeles blancos del mapa (libre) y $\theta^{[i]} \sim \mathcal{U}(-\pi, \pi)$.

Esto representa **ignorancia total** sobre la posición: hipótesis global de localización.

---

### Paso G — Dead Reckoning con odometría

En cada timestep, se calcula el desplazamiento del robot en su **propio frame** a partir del delta de odometría:

Sea $\Delta x_w, \Delta y_w$ el desplazamiento en el frame del mundo y $\theta_{t-1}$ la orientación anterior:

$$\Delta x_r = \Delta x_w \cos\theta_{t-1} + \Delta y_w \sin\theta_{t-1}$$
$$\Delta y_r = -\Delta x_w \sin\theta_{t-1} + \Delta y_w \cos\theta_{t-1}$$
$$\Delta\theta = \theta_t - \theta_{t-1}$$

---

### Paso H — Propagación de partículas (Modelo de movimiento)

Cada partícula se actualiza aplicando el mismo desplazamiento estimado más **ruido gaussiano**:

$$x^{[i]}_{t} = x^{[i]}_{t-1} + (\Delta x_r \cos\theta^{[i]} - \Delta y_r \sin\theta^{[i]}) + \mathcal{N}(0, \sigma_{xy})$$
$$y^{[i]}_{t} = y^{[i]}_{t-1} + (\Delta x_r \sin\theta^{[i]} + \Delta y_r \cos\theta^{[i]}) + \mathcal{N}(0, \sigma_{xy})$$
$$\theta^{[i]}_{t} = \theta^{[i]}_{t-1} + \Delta\theta + \mathcal{N}(0, \sigma_\theta)$$

El ruido representa la incertidumbre del modelo de movimiento (deslizamiento, encoders imperfectos).

---

### Paso E — Puntuación de partículas (Modelo de observación)

Para puntuar cada partícula se usa el **Likelihood Field Model** (Thrun §6.4), que es más robusto que comparar pixeles directamente.

#### Construcción del campo de probabilidad

Dado el mapa binario (pared/libre), se calcula para cada pixel su distancia al **pixel de pared más cercano** usando `cv2.distanceTransform`. Luego se aplica una Gaussiana:

$$\ell(c, r) = \exp\!\left(-\frac{d_{\text{wall}}(c,r)^2}{2\sigma_L^2}\right)$$

donde $d_{\text{wall}}$ es la distancia en metros al pixel de pared más cercano y $\sigma_L = 0.10$ m controla la tolerancia.

| Ubicación del punto | $d_{\text{wall}}$ | Score $\ell$ |
|---|---|---|
| Justo en la pared | ≈ 0 | ≈ 1.0 |
| 5 cm dentro del espacio libre | 0.05 m | ≈ 0.88 |
| 20 cm dentro del espacio libre | 0.20 m | ≈ 0.14 |
| Fuera del mapa | — | 0.0 |

#### Puntuación de una partícula

Para cada rayo del LIDAR con rango $r_k$ y ángulo relativo $\alpha_k$, el endpoint en el mundo es:

$$e_x^k = x^{[i]} + r_k \cos(\theta^{[i]} + \alpha_k)$$
$$e_y^k = y^{[i]} + r_k \sin(\theta^{[i]} + \alpha_k)$$

Se convierte a coordenadas de pixel y se consulta el campo:

$$\text{score}^{[i]} = \sum_{k} \ell\!\left(\text{col}(e_x^k),\, \text{row}(e_y^k)\right)$$

Una partícula en la pose correcta producirá endpoints alineados con las paredes → score alto. Una partícula desplazada tendrá endpoints en el espacio libre → score bajo.

---

### Paso F — Filtrado y resampling

Los pesos normalizados son:

$$w^{[i]} = \frac{\text{score}^{[i]}}{\sum_j \text{score}^{[j]}}$$

Se realiza **systematic resampling**: se sortean $N$ índices con probabilidad proporcional a $w^{[i]}$. Partículas con alto peso son copiadas múltiples veces; las de bajo peso desaparecen.

Se añade ruido pequeño tras el resampling para evitar **colapso del filtro** (todas las partículas en un único punto):

$$\mathbf{x}^{[i]} \leftarrow \mathbf{x}^{[i]} + \mathcal{N}(\mathbf{0},\, \Sigma_{\text{resample}})$$

---

### Paso I — Loop

El ciclo completo se ejecuta en cada mensaje `/scan` (~10 Hz):

```
┌──────────────────────────────────────────────┐
│ /scan llega                                  │
│   ↓                                          │
│ G: calcular delta odom                       │
│   ↓                                          │
│ H: propagar partículas (dead reckoning)      │
│   ↓                                          │
│ E: puntuar partículas (likelihood field)     │
│   ↓                                          │
│ F: resamplear (sistemático + noise)          │
│   ↓                                          │
│ Publicar /particle_cloud y /mcl_pose         │
└──────────────────────────────────────────────┘
       ↑____________________vuelve al inicio___┘
```

---

## Parámetros ajustables

Todos están como constantes de clase al inicio de `mcl_node.py`:

| Parámetro | Valor actual | Efecto |
|-----------|-------------|--------|
| `N_PARTICLES` | 800 | Más partículas = más precisión, más CPU |
| `BEAM_STRIDE` | 8 | Cada cuántos rayos usar (360/8 = 45 beams) |
| `LIDAR_SIGMA` | 0.10 m | Tolerancia del likelihood field. Más grande = más permisivo |
| `NOISE_XY` | 0.03 m | Ruido post-resample (diversidad de partículas) |
| `NOISE_THETA` | 0.03 rad | Ruido angular post-resample |
| `MOTION_NOISE_XY` | 0.005 m | Ruido del modelo de movimiento |
| `MOTION_NOISE_THETA` | 0.005 rad | Ruido angular del modelo de movimiento |
| `MAX_RANGE` | 4.5 m | Rango máximo del LIDAR a considerar |

---

## Diferencias vs. la actividad original

La actividad pide "sumas de valores de pixeles" (paso E) como función de scoring. Esta implementación usa el **Likelihood Field** en su lugar porque:

- El endpoint de un rayo LIDAR cae en el **último pixel libre** antes de la pared — con lookup directo ese pixel es blanco (255) → score 0, que es incorrecto.
- El Likelihood Field convierte la distancia a la pared más cercana en un score gaussiano, lo que es matemáticamente equivalente al espíritu de la actividad pero robusto a errores de 1–2 pixeles.

La lógica central (partículas → score → filtrar → dead reckoning → loop) sigue exactamente los pasos A–I.
