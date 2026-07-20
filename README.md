# ros2_robot_evasor

Simulación en ROS 2 Gazebo de un robot móvil diferencial con navegación autónoma, evasión de obstáculos y mapeo SLAM en tiempo real.

## Requisitos

- ROS 2 **Jazzy**
- Gazebo **Gz Sim** (incluido con ROS 2 Jazzy)
- Paquetes Nav2 (`nav2_bringup`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_velocity_smoother`, `nav2_collision_monitor`, `nav2_mppi_controller`, `nav2_navfn_planner`, `nav2_lifecycle_manager`, `nav2_rviz_plugins`)
- `slam_toolbox`
- `ros_gz_sim`, `ros_gz_bridge`
- `robot_state_publisher`, `xacro`

## Instalación

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_robot_evasor
colcon build
source install/setup.bash
```

## Uso

```bash
ros2 launch robot_evasor_bringup bringup.launch.py

__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia   ros2 launch robot_evasor_bringup bringup.launch.py diagnostics:=true #En case de querer usar GPU
```

Esto inicia Gazebo con el mundo simulado, el robot, el stack completo de Nav2, SLAM Toolbox y RViz.

### En RViz

1. **2D Pose Estimate** — haz clic en el mapa donde está el robot para darle la pose inicial.
2. **Nav2 Goal** — haz clic en cualquier punto del mapa para enviar un destino. El robot planificará una ruta, esquivará obstáculos y navegará hasta allí.

## Robot — evasor_bot

Robot diferencial de 4 ruedas con Lidar 360°.

| Componente | Especificación |
|---|---|
| Chasis | 0.35 × 0.20 × 0.08 m |
| Ruedas traseras (tracción) | radio 0.04 m, ancho 0.03 m, fricción alta |
| Ruedas delanteras (locas) | radio 0.04 m, ancho 0.03 m, fricción baja |
| Separación entre ruedas | 0.24 m |
| Lidar | GPU Lidar, 360°, 5 m de alcance, 10 Hz |
| Velocidad máxima | 0.5 m/s |

### Árbol TF

```
map → odom → base_footprint → base_link → ruedas, lidar
```

## Entorno simulado

Habitación de 10 × 10 m con 4 paredes y 5 obstáculos distribuidos (cajas y cilindro). Definido en [`src/robot_evasor_gazebo/worlds/room.world`](src/robot_evasor_gazebo/worlds/room.world).

## Arquitectura

```
Gazebo Sim (room.world + evasor_bot)
    │
    ├── /scan, /odometry, /joint_states  ──→  ros_gz_bridge  ──→  ROS 2 topics
    │
    ├── robot_state_publisher  ──→  TF joint → base_link
    │
    ├── odom_to_tf.py  ──→  TF odom → base_footprint
    │
    └── Nav2 Stack:
         ├── SLAM Toolbox  ──→  /map, TF map → odom
         ├── AMCL  ──→  localización por partículas
         ├── Planner Server (Navfn)  ──→  ruta global
         ├── Controller Server (MPPI)  ──→  control local + cmd_vel
         ├── Velocity Smoother  ──→  suavizado de velocidades
         ├── Collision Monitor  ──→  parada de emergencia
         └── BT Navigator  ──→  orquestación de navegación
              ↑
    RViz (GoalTool → /goal_pose)
```

## Paquetes

| Paquete | Propósito |
|---|---|
| `robot_evasor_bringup` | Punto de entrada: lanza Gazebo + Nav2 + RViz |
| `robot_evasor_control` | Nodos Nav2, SLAM, odom_to_tf, configuración |
| `robot_evasor_description` | Modelo URDF (xacro), configuración de RViz |
| `robot_evasor_gazebo` | Mundo Gazebo, bridge ROS-Gazebo, spawn del robot |

## Archivos de configuración

| Archivo | Propósito |
|---|---|
| `src/robot_evasor_control/config/nav2_params.yaml` | Parámetros de Nav2 (AMCL, costmaps, controladores) |
| `src/robot_evasor_control/config/slam_params.yaml` | Parámetros de SLAM Toolbox |
| `src/robot_evasor_gazebo/config/bridge.yaml` | Puente de topics entre ROS y Gazebo |

## Notas

- `odom_to_tf.py` es crítico: Gazebo publica odometría pero no genera la TF `odom → base_footprint` que Nav2 necesita.
- SLAM y Nav2 usan lifecycle managers separados para evitar bloqueos mutuos.
- El nodo de diagnóstico (`diagnostics_node.py`) está deshabilitado por defecto; actívalo con `diagnostics:=true`.
