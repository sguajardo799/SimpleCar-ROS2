# Simulador 2D para ROS 2

Simulador liviano de un vehiculo sobre pistas editables. Usa un unico nodo ROS 2 y publica `/scan`, `/odom` y `/goal_reached`; recibe movimiento mediante `/cmd_vel`.

## Instalar dependencias

Antes de cualquier comando ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Instala las dependencias del paquete:

```bash
sudo apt update
sudo apt install python3-pygame python3-numpy ros-jazzy-ament-index-python
```

## Compilar

Desde este directorio:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Ejecutar y seleccionar nivel

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch track_sim simulator.launch.py level:=level2
```

Estan incluidos `level1`, `level2` y `level3`. Para probar con teclado:

```bash
ros2 launch track_sim simulator.launch.py level:=level1 keyboard_control:=true
```

Controles: `W`/`S` o flechas verticales para avanzar y retroceder, `A`/`D` o flechas laterales para girar, `R` para reiniciar y `ESC` para salir. En modo teclado se ignora `/cmd_vel` para evitar mezclar controles.

Tambien puede ejecutarse sin launch:

```bash
ros2 run track_sim simulator --ros-args -p level:=level3 -p keyboard_control:=false
```

## Probar los topicos

En otra terminal, carga ROS 2 y el workspace antes de cada prueba:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.6}, angular: {z: 0.0}}"
```

Para observar el pseudo-LiDAR y la odometria:

```bash
ros2 topic echo /scan
ros2 topic echo /odom
```

El nivel se reinicia con `R` o mediante el servicio:

```bash
ros2 service call /reset std_srvs/srv/Empty
```

## Crear un nivel

Crea dos archivos con el mismo nombre dentro de `track_sim/levels/`. No es necesario modificar Python.

`level4.csv` define la linea central y el ancho total de la pista en metros:

```csv
x,y,width
0.0,0.0,2.0
2.0,0.0,2.0
4.0,1.0,1.8
6.0,2.0,1.8
```

`level4.txt` define inicio y meta:

```text
start_x=0.5
start_y=0.0
start_yaw=0.0
goal_x=5.5
goal_y=1.8
goal_radius=0.4
```

Vuelve a ejecutar `colcon build` para instalar los archivos nuevos y lanza `level:=level4`.
