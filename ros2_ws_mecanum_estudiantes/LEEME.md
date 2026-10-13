# Examen Robótica Móvil — Robot Mecanum (ROS 2 Jazzy + Gazebo Harmonic)

El workspace de esta copia está configurado para la **variante C** del examen.

| Paquete | Contenido |
|---|---|
| `mi_robot_mecanum` | urdf, mundo `pista`, launch, rviz |
| `mi_mecanum_control` | nodos `cinematica` (W, W⁺, odometría) e `ir_a_puntos` (ec. 21) |

## Compilar

```zsh
# Limpiar referencias a otros workspaces heredadas por esta terminal
unset COLCON_CURRENT_PREFIX COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.zsh
cd ~/Documents/parcial3/ros2_ws_mecanum_estudiantes/ros2_ws_mecanum_estudiantes/ros2_ws_mecanum
colcon build --symlink-install
source install/setup.zsh
ros2 pkg prefix mi_mecanum_control
```

## Ejecutar

```zsh
# Terminal 1: Gazebo + robot + bridge + cinemática + RViz
source /opt/ros/jazzy/setup.zsh
cd ~/Documents/parcial3/ros2_ws_mecanum_estudiantes/ros2_ws_mecanum_estudiantes/ros2_ws_mecanum
source install/setup.zsh
ros2 launch mi_robot_mecanum simulacion.launch.py
```

```zsh
# Terminal 2: controlador de waypoints (ejecutar solo una instancia)
source /opt/ros/jazzy/setup.zsh
cd ~/Documents/parcial3/ros2_ws_mecanum_estudiantes/ros2_ws_mecanum_estudiantes/ros2_ws_mecanum
source install/setup.zsh
ros2 run mi_mecanum_control ir_a_puntos --ros-args -p use_sim_time:=true
```

La teleoperación con teclado es una alternativa manual; no la ejecute simultáneamente con `ir_a_puntos`.

## Archivos configurados

| Archivo | Parte |
|---|---|
| `mi_robot_mecanum/urdf/mi_robot.urdf.xacro` | Ruedas, motores, encoders, LiDAR y Astra S |
| `mi_robot_mecanum/worlds/pista.sdf` | Sistema de sensores |
| `mi_robot_mecanum/launch/simulacion.launch.py` | Bridge, lanzamiento y spawn |
| `mi_robot_mecanum/rviz/control.rviz` | Displays RViz requeridos |
| `mi_mecanum_control/mi_mecanum_control/cinematica.py` | W, W⁺, cinemática inversa/directa y odometría |
| `mi_mecanum_control/mi_mecanum_control/ir_a_puntos.py` | Controlador de waypoints C |

## Cálculos aplicados para la variante C

### Geometría y cinemática

- Radio de rueda: `a = 0.040 m`.
- Wheelbase: `0.26 m`, por tanto `lx = 0.26/2 = 0.13 m`.
- Track: `0.24 m`, por tanto `ly = 0.24/2 = 0.12 m`.
- `lx + ly = 0.25 m`.
- LiDAR: `16000 muestras/s ÷ 5 vueltas/s = 3200 muestras/vuelta`.
- Astra S: FOV horizontal de profundidad `58.4° × π/180 = 1.019054 rad`; imagen `640 × 480`; rango de profundidad `0.4–2.0 m`; actualización `30 Hz`.
- RPLIDAR A3 Enhanced: rango `0.2–25 m`, barrido `5 Hz`, ruido gaussiano con `σ = 0.01 m`.
- Posiciones respecto a `base_link`: LiDAR `(-0.03, 0, 0.15) m`; Astra S `(0.12, 0, 0.07) m`.

La matriz de cinemática directa que usa el nodo es:

```text
W = 0.01 * [[ 1,  1,  1,  1],
            [-1,  1,  1, -1],
            [-4,  4, -4,  4]]
```

La pseudoinversa (filas en orden `del_izq`, `del_der`, `tra_izq`, `tra_der`) es:

```text
W+ = [[25, -25, -6.25],
      [25,  25,  6.25],
      [25,  25, -6.25],
      [25, -25,  6.25]]
```

Los waypoints cargados son `(1, 1, π/4) → (2, 0, 0) → (1, −1, −π/4) → (0, 0, 0)`. El controlador transforma el error de posición de `odom` al marco del robot con `J(φ)⁺ = J(φ)ᵀ`, limita la norma de velocidad planar a `0.4 m/s` y la velocidad angular a `1.0 rad/s`.

Las inercias se modelan con sólidos uniformes y dimensiones externas. Para cada rueda cilíndrica (`m=0.30 kg`, `r=0.04 m`, `h=0.04 m`), `Ixx=Iyy=0.000160` e `Izz=0.000240 kg·m²`. Para el RPLIDAR A3 (`Ø76 mm × 41 mm`, `0.19 kg`), `Ixx=Iyy≈0.0000952` e `Izz≈0.0001372 kg·m²`. Para la Astra S (caja de `40 × 165 × 30 mm`, profundidad × ancho × alto, `0.31 kg`), `Ixx≈0.0007266`, `Iyy≈0.0000646` e `Izz≈0.0007446 kg·m²`. Son aproximaciones de carcasa sólida para URDF.

Fuentes: [RPLIDAR A3 — SLAMTEC](https://www.slamtec.com/en/lidar/a3spec), [Astra Series — Orbbec](https://www.orbbec.com/products/structured-light-camera/astra-series/), [Astra S datasheet](https://www.mybotshop.de/Datasheet/Orbbec_Astra_S.pdf).

**No modifique** `launch/sdf_mecanum.py` ni la `<collision>`.

## Depuración

```bash
xacro src/mi_robot_mecanum/urdf/mi_robot.urdf.xacro > /tmp/check.urdf && echo OK
gz topic -l
ros2 topic list
ros2 topic echo /joint_states --once
ros2 node info /cinematica
```
