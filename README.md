# Robotics Control Lab 4.2  
## Control de Posición del xArm Lite 6 bajo Perturbaciones Sinusoidales y Gaussianas

**Materia:** TE3001B - Fundamentación de Robótica
**Profesor:** Nezih Nieto Gutiérrez - Modulo 4 Control de Robots  
**Institución:** Tecnológico de Monterrey  
**Equipo:** 2 - REPO  
**Fecha:** 25 de Febrero del 2026  

---

# Descripción del Proyecto

Este proyecto evalúa el desempeño de tres controladores PD cartesianos aplicados al robot xArm Lite 6 bajo diferentes condiciones de perturbación:

- Condición base (sin perturbación)
- Perturbación sinusoidal (determinista)
- Perturbación gaussiana (estocástica)

Se analizaron dos trayectorias distintas:

- `circle_maker.py`
- `triangle_maker.py`

Se realizaron un total de **36 experimentos** para evaluar estabilidad, precisión y sensibilidad al ruido.

---

# Objetivo

Comparar tres configuraciones distintas de controladores PD y analizar su desempeño en términos de:

- RMSE por eje
- RMSE total
- Error absoluto máximo
- Comportamiento de saturación de velocidad
- Sensibilidad ante perturbaciones seno y gauss

---

# Controladores Evaluados

| Controlador | Kp | Kd | Característica |
|------------|----|----|---------------|
| C1 | 5.5 | 0.15 | Suave, menos agresivo |
| C2 | 7.0 | 0.30 | Balanceado, mejor desempeño global |
| C3 | 5.0 | 0.45 | Alto amortiguamiento, sensible al ruido |

Todos los controladores incluyen:

- Implementación de deadband
- Saturación de velocidad

Toda práctica se realizó bajo límites seguros de nuestro espacio de trabajo, con dos personas siempre trabajando con el robot y/o estando al tanto, almenos una persona, siempre con el robot al ejecutar código que genere movimiento.

---

# Ley de Control

El controlador PD cartesiano utilizado es:

\[
v = K_p \cdot e + K_d \cdot \dot{e}
\]

Donde:

- \( K_p \) actúa como rigidez
- \( K_d \) actúa como amortiguamiento
- Se aplica deadband para evitar micro-oscilaciones
- Se limita la magnitud de velocidad (saturación)

---

# Experimentos Realizados

Cada controlador fue evaluado bajo:

## Trayectoria Circular
- Baseline (sin perturbación)
- Seno: 0.01, 0.02, 0.03
- Gauss: 0.01, 0.02, 0.03

## Trayectoria Triangular
- Baseline
- Seno: 0.01, 0.02, 0.03
- Gauss: 0.01, 0.02, 0.03

Dando un total de **36 experimentos**

---

# Métricas Calculadas

Para cada experimento se calcularon:

- RMSE por eje
- RMSE total
- Error absoluto máximo
- Magnitud de velocidad comandada
- Evolución temporal del error

Los datos fueron procesados mediante scripts en Python y MATLAB.

---

---

# Cómo Ejecutar el Proyecto

## 0 - Directorio y Source

```bash
cd ~/xarm_ws
source install/setup.bash
```

## 1️ - Lanzar MoveIt y Servo

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.175
```

## 2 - Ejecutar la Trajectoria

```bash
ros2 run xarm_perturbations circle_maker --ros-args   -p publish_twist:=false   -p desired_topic:=/desired_position   -p radius:=0.06 -p frequency:=0.06 -p plane:=xy -p hold_z:=false

ros2 run xarm_perturbations triangle_maker --ros-args   -p amplitude:=0.06   -p frequency:=0.1   -p z_height:=0.207
```

## 3 - Ejecutar el Controlador

```bash
ros2 run xarm_perturbations position_controller --ros-args   -p target_topic:=/desired_position   -p output_topic:=/controller/delta_twist_cmds   -p kp:="[5.0, 5.0, 5.0]"   -p kd:="[0.45, 0.45, 0.45]"   -p ki:="[0.0, 0.0, 0.0]"   -p max_speed:=0.18   -p deadband:=0.003
```
## 4 - Ejecutar la Perturbación

```bash
ros2 run xarm_perturbations perturbation_injector --ros-args   -p input_topic:=/controller/delta_twist_cmds   -p output_topic:=/servo_server/delta_twist_cmds   -p enabled:=true -p mode:=sine   -p sine_freq_hz:=1.0 -p sine_amp_linear:=0.01 -p sine_axis:=x   -p base_linear:="[0.0, 0.0, 0.0]"   -p debug:=true   -p pub_reliability:=reliable

ros2 run xarm_perturbations perturbation_injector --ros-args   -p input_topic:=/controller/delta_twist_cmds   -p output_topic:=/servo_server/delta_twist_cmds   -p enabled:=true -p mode:=gaussian   -p gauss_std_linear:=0.1   -p gauss_axis:=x   -p base_linear:="[0.0, 0.0, 0.0]"   -p debug:=true   -p pub_reliability:=reliable
```

# Obtención de datos

## 1 - Guardar los datos crudos en raw_log.txt
## 2 - Ejecutar analyze_logs.py (Obtener parsed_data.csv)

```bash
python analyze_logs.py
```

## 3 - Ejecutar controller_analyze.m (Obtener gráficas)

```bash
controller_analyze.m
```

# Principales observaciones (en resumen)

- El controlador C2 (Kp=7.0, Kd=0.3) mostró el mejor desempeño global.
- El controlador C3 amplifica ruido gaussiano debido al alto término derivativo.
- El controlador C1 es el más suave y estable, pero con mayor error de seguimiento.
- Las perturbaciones sinusoidales fueron más fáciles de compensar que las gaussianas.

 # Requisitos

- Ubuntu / Virtual Machine Ubuntu / Cualquier versión
- ROS 2 Humble / ROS 2 Jazzy / Dependiendo de versión de Ubuntu
- MoveIt 2
- xArm Lite 6
- Python 3
- MATLAB (opcional; se puede usar Python 3)

# Autores

Manuel Ferro Sánchez, 
Zacbe Ortega Obregón, 
Fabricio Banda Hernández, 
Alexandro Kurt Cárdenas Pérez, 
Fernando Proal Sifuentes.
