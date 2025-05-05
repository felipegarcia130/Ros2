# Ros2 🧠🤖

Repositorio profesional con múltiples paquetes y proyectos desarrollados en ROS 2, orientados al aprendizaje, experimentación y desarrollo de soluciones en robótica móvil, simulación y control.

Este repositorio reúne implementaciones tanto en C++ como en Python, incluyendo simulaciones en Gazebo, controladores personalizados, ejemplos educativos, y paquetes avanzados de navegación.

---

## 📁 Estructura del repositorio

| Carpeta                        | Descripción breve                                                                 |
|-------------------------------|------------------------------------------------------------------------------------|
| `chall_2/`                    | Desafíos prácticos con movimiento y navegación básica.                            |
| `cpp_pubsub/`, `cpp_srvcli/`  | Nodos básicos en C++ para comunicación `publisher/subscriber` y `services`.      |
| `py_pubsub/`, `py_srvcli/`    | Equivalente en Python para prácticas introductorias de ROS 2.                    |
| `ros_tutorials/`              | Ejercicios de aprendizaje para entender el ecosistema de ROS 2.                  |
| `differential_drive/`         | Implementación de modelos cinemáticos y controladores de robots diferenciales.   |
| `gazebo_puzzlebot_sim/`       | Simulación del Puzzlebot en Gazebo, con sensores y escenarios personalizables.   |
| `line_tracer_cmdvel/`         | Controlador para seguir líneas en el entorno simulado, usando `cmd_vel`.         |
| `omni3_controller_pkg/`       | Controlador de cinemática para plataforma omnidireccional de 3 ruedas.           |
| `micro_ros_setup/`            | Integración de micro-ROS para trabajar con microcontroladores y ROS 2.           |
| `signal_processing/`          | Procesamiento de señales aplicado a sensores o entradas de control.              |
| `turtle_tf2_broadcaster/`     | Ejemplo de broadcasting de frames TF2 para visualización en RViz.                |
| `puzzlebot_sim/`              | Simulación base del Puzzlebot para pruebas de navegación y control.              |
| `examples/`, `Activity 3/`    | Prácticas y experimentos adicionales con sensores, movimientos y visión.         |

---

## 🔧 Requisitos

- ROS 2 Humble (Ubuntu 22.04)
- Gazebo (Ignition o clásico)
- Python 3.10+
- colcon / CMake

---

## 🚀 Compilación

```bash
cd ~/ros2_ws
colcon build 
source install/setup.bash
