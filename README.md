# ROS 2 Robotics Repository 🤖 🧠

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()

Repositorio profesional con múltiples paquetes y proyectos desarrollados en ROS 2, orientados al aprendizaje, experimentación y desarrollo de soluciones en robótica móvil, simulación y control.

Este repositorio reúne implementaciones tanto en C++ como en Python, incluyendo simulaciones en Gazebo, controladores personalizados, ejemplos educativos, y paquetes avanzados de navegación.

## 📋 Contenido

- [Estructura del Repositorio](#-estructura-del-repositorio)
- [Requisitos](#-requisitos)
- [Instalación](#-instalación)
- [Compilación](#-compilación)
- [Ejecución de Ejemplos](#-ejecución-de-ejemplos)
- [Contribuciones](#-contribuciones)
- [Licencia](#-licencia)

## 📁 Estructura del Repositorio

| Carpeta | Descripción |
|---------|-------------|
| `chall_2/` | Desafíos prácticos con movimiento y navegación básica |
| `cpp_pubsub/`, `cpp_srvcli/` | Nodos básicos en C++ para comunicación `publisher/subscriber` y `services` |
| `py_pubsub/`, `py_srvcli/` | Equivalente en Python para prácticas introductorias de ROS 2 |
| `ros_tutorials/` | Ejercicios de aprendizaje para entender el ecosistema de ROS 2 |
| `differential_drive/` | Implementación de modelos cinemáticos y controladores de robots diferenciales |
| `gazebo_puzzlebot_sim/` | Simulación del Puzzlebot en Gazebo, con sensores y escenarios personalizables |
| `line_tracer_cmdvel/` | Controlador para seguir líneas en el entorno simulado, usando `cmd_vel` |
| `omni3_controller_pkg/` | Controlador de cinemática para plataforma omnidireccional de 3 ruedas |
| `micro_ros_setup/` | Integración de micro-ROS para trabajar con microcontroladores y ROS 2 |
| `signal_processing/` | Procesamiento de señales aplicado a sensores o entradas de control |
| `turtle_tf2_broadcaster/` | Ejemplo de broadcasting de frames TF2 para visualización en RViz |
| `puzzlebot_sim/` | Simulación base del Puzzlebot para pruebas de navegación y control |
| `examples/`, `Activity 3/` | Prácticas y experimentos adicionales con sensores, movimientos y visión |

## 🔧 Requisitos

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- Gazebo (Ignition Fortress o Gazebo Classic)
- Python 3.10+
- colcon / CMake

## 📥 Instalación

### 1. Instalación de ROS 2 Humble

```bash
# Configurar repositorios
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Configurar llaves
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Instalar dependencias adicionales
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup python3-colcon-common-extensions
```

### 2. Clonar este repositorio

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/felipegarcia130/ros2.git
```

## 🚀 Compilación

```bash
# Navegar al workspace
cd ~/ros2_ws

# Compilar todos los paquetes
colcon build

# Fuente del entorno
source install/setup.bash
```

Para compilar paquetes específicos:

```bash
colcon build --packages-select <nombre_paquete>
```

## 🎮 Ejecución de Ejemplos

### Simulación del Puzzlebot en Gazebo

```bash
# Terminal 1: Lanzar la simulación
source ~/ros2_ws/install/setup.bash
ros2 launch puzzlebot_sim puzzlebot_gazebo.launch.py

# Terminal 2: Controlar el robot
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Ejemplo de Publisher-Subscriber

```bash
# Terminal 1: Ejecutar el talker (C++)
source ~/ros2_ws/install/setup.bash
ros2 run cpp_pubsub talker

# Terminal 2: Ejecutar el listener (Python)
source ~/ros2_ws/install/setup.bash
ros2 run py_pubsub listener
```

### Seguidor de Línea

```bash
# Terminal 1: Lanzar la simulación
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_puzzlebot_sim line_world.launch.py

# Terminal 2: Ejecutar el controlador
source ~/ros2_ws/install/setup.bash
ros2 run line_tracer_cmdvel line_follower
```

## 📂 Estructura de Paquetes Destacados

### 1. differential_drive

Implementación de controladores para robots de tracción diferencial:

- Control cinemático y dinámico
- Cálculo de odometría
- Transformaciones de coordenadas

### 2. signal_processing

Algoritmos para procesamiento de señales sensoriales:

- Filtros para reducción de ruido
- Detección de características
- Fusión sensorial

## 🤝 Contribuciones

Las contribuciones son bienvenidas. Por favor, sigue estos pasos:

1. Fork este repositorio
2. Crea una rama con tu característica (`git checkout -b feature/amazing-feature`)
3. Realiza tus cambios (`git commit -m 'Add some amazing feature'`)
4. Push a la rama (`git push origin feature/amazing-feature`)
5. Abre un Pull Request

## 📄 Licencia

Este proyecto está licenciado bajo los términos de la licencia MIT. Consulta el archivo [LICENSE](LICENSE) para más detalles.

---

Desarrollado con ❤️ por [Felipe García](https://github.com/felipegarcia130)
