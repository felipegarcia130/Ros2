# ROS 2 — Robotics Repository

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Repositorio compartido entre dos máquinas. Reúne implementaciones de navegación autónoma para el robot diferencial **Puzzlebot** (Manchester Robotics), así como prácticas educativas, simulaciones en Gazebo y controladores en C++ y Python.

---

## Contenido

- [Paquetes — Máquina de navegación (Puzzlebot físico)](#paquetes--máquina-de-navegación-puzzlebot-físico)
- [Paquetes — Máquina educativa / simulación](#paquetes--máquina-educativa--simulación)
- [Requisitos](#requisitos)
- [Instalación](#instalación)
- [Compilación](#compilación)
- [Ejecución](#ejecución)
- [Licencia](#licencia)

---

## Paquetes — Máquina de navegación (Puzzlebot físico)

### `mcl_robot` — Localización, Planificación y SLAM

| Nodo | Descripción |
|------|-------------|
| `mcl_node` | **Localización MCL + EKF de respaldo.** Filtro de partículas con muestreo KLD-Sampling (N adaptativo), modelo de movimiento robusto (RAM con Metropolis-Hastings) y selección adaptativa del modelo de sensor mediante un bandit UCB. Máquina de estados de 3 fases (MCL → EKF → SYNC) cuando MCL pierde convergencia. |
| `astar_node` | **Planificación A* + navegación.** Genera rutas en el mapa de ocupación con inflado de obstáculos. Sigue waypoints con control proporcional y evasión reactiva con LiDAR. Acepta goals desde RViz (`/goal_pose`). |
| `exploration_node` | **Exploración autónoma por fronteras.** Detecta fronteras entre celdas conocidas y desconocidas, selecciona el cluster más cercano y planifica con A*. |
| `slam_node` | **SLAM 2D con log-odds.** Construye un mapa de ocupación en tiempo real con LiDAR y odometría (trazado de rayos Bresenham). Controlado por teclado. |
| `odom_node` | **Odometría diferencial.** Lee velocidades de encoders (`/VelocityEncL`, `/VelocityEncR`) e integra la cinemática diferencial para publicar `/odom` y el TF `odom → base_link`. |

**Topics principales**

| Topic | Tipo | Dirección |
|-------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | entrada |
| `/odom` | `nav_msgs/Odometry` | entrada |
| `/mcl_pose` | `geometry_msgs/PoseStamped` | salida MCL |
| `/cmd_vel` | `geometry_msgs/Twist` | salida control |
| `/map` | `nav_msgs/OccupancyGrid` | salida mapa |
| `/goal_pose` | `geometry_msgs/PoseStamped` | entrada goal (RViz) |
| `/planned_path` | `nav_msgs/Path` | salida ruta A* |

**Arquitectura**

```
LiDAR (/scan) ──────────────────────────────────┐
                                                 ▼
Encoders → odom_node → /odom → mcl_node → /mcl_pose → astar_node → /cmd_vel
                                    │                        ▲
                                    └── EKF backup ──────────┘
                                        (cuando MCL falla)
```

---

### `obstacle_avoidance` — Algoritmos Bug

Implementaciones de los tres algoritmos Bug para navegación reactiva. Todos consumen la pose de MCL (`/mcl_pose`) y publican en `/cmd_vel`.

| Nodo | Algoritmo | Descripción |
|------|-----------|-------------|
| `bug0` | Bug 0 | Va al goal en línea recta; bordea la pared hacia el lado más libre si hay obstáculo. Incluye filtro de seguridad reactivo. |
| `bug1` | Bug 1 | Rodea el obstáculo completo, memoriza el punto más cercano al goal y retrocede hasta él. |
| `bug2` | Bug 2 | Usa la línea M (start → goal); abandona el bordeo al reintersectar la línea M más cerca del goal. |

---

### `puzzle_sim` — Simulación Gazebo

Modelos SDF/URDF del Puzzlebot con launch file para Gazebo.

```bash
ros2 launch puzzle_sim gazebo.launch.py
```

---

### `mi_servicio` — Ejemplo servicio ROS 2

Ejemplo básico de servidor/cliente con servicios ROS 2 en Python.

---

### `mi_interfaces` — Interfaces personalizadas

Definiciones de mensajes y servicios custom usados por los paquetes del workspace.

---

### `azure_kinect_ros_driver` — Driver Kinect Azure

Driver C++ para la cámara de profundidad Azure Kinect (K4A). Publica nubes de puntos, imágenes RGB y datos de profundidad.

---

## Paquetes — Máquina educativa / simulación

| Carpeta | Descripción |
|---------|-------------|
| `chall_2/` | Desafíos prácticos con movimiento y navegación básica |
| `cpp_pubsub/`, `cpp_srvcli/` | Nodos básicos en C++ para `publisher/subscriber` y `services` |
| `py_pubsub/`, `py_srvcli/` | Equivalente en Python para prácticas introductorias |
| `ros_tutorials/` | Ejercicios de aprendizaje del ecosistema ROS 2 |
| `differential_drive/` | Modelos cinemáticos y controladores de robots diferenciales |
| `gazebo_puzzlebot_sim/` | Simulación del Puzzlebot en Gazebo con sensores y escenarios |
| `line_tracer_cmdvel/` | Controlador para seguir líneas usando `cmd_vel` |
| `omni3_controller_pkg/` | Cinemática para plataforma omnidireccional de 3 ruedas |
| `micro_ros_setup/` | Integración de micro-ROS con microcontroladores |
| `signal_processing/` | Procesamiento de señales: filtros, detección de características, fusión sensorial |
| `turtle_tf2_broadcaster/` | Broadcasting de frames TF2 para visualización en RViz |
| `puzzlebot_sim/` | Simulación base del Puzzlebot para pruebas de navegación |
| `examples/`, `Activity 3/` | Prácticas con sensores, movimientos y visión |

---

## Requisitos

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- Python 3.10+
- Gazebo Classic
- `colcon` / CMake
- Dependencias Python (máquina de navegación): `numpy`, `opencv-python`, `scipy`, `pynput`

```bash
pip install numpy opencv-python scipy pynput
```

---

## Instalación

### 1. Instalar ROS 2 Humble

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-navigation2 \
     ros-humble-nav2-bringup python3-colcon-common-extensions
```

### 2. Clonar el repositorio

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/felipegarcia130/ros2.git
```

---

## Compilación

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Para compilar solo un paquete:

```bash
colcon build --packages-select <nombre_paquete>
```

---

## Ejecución

### Localización MCL + Navegación A* (robot físico)

```bash
# Terminal 1
ros2 run mcl_robot odom_node

# Terminal 2
ros2 run mcl_robot mcl_node

# Terminal 3 — poner el goal con "2D Goal Pose" en RViz
ros2 run mcl_robot astar_node
```

### SLAM interactivo

```bash
ros2 run mcl_robot slam_node
```

### Exploración autónoma

```bash
ros2 run mcl_robot mcl_node          # Terminal 1
ros2 run mcl_robot exploration_node  # Terminal 2
```

### Algoritmos Bug (requieren mcl_node activo)

```bash
ros2 run obstacle_avoidance bug0   # o bug1 / bug2
```

### Simulación del Puzzlebot en Gazebo

```bash
ros2 launch puzzle_sim gazebo.launch.py

# Controlar con teclado
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Ejemplo Publisher-Subscriber

```bash
ros2 run cpp_pubsub talker      # Terminal 1
ros2 run py_pubsub listener     # Terminal 2
```

### Seguidor de línea

```bash
ros2 launch gazebo_puzzlebot_sim line_world.launch.py
ros2 run line_tracer_cmdvel line_follower
```

---

## Licencia

MIT — ver [LICENSE](LICENSE).

---

Desarrollado por [Felipe García](https://github.com/felipegarcia130)
