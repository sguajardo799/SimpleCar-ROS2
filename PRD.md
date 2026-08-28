# PRD — Simulador 2D de Vehículo para Laboratorio Introductorio ROS 2

## 1. Objetivo

Desarrollar un simulador 2D liviano para ROS 2 Jazzy que represente un vehículo desplazándose sobre una pista definida por archivos externos editables.

El simulador será utilizado en un primer laboratorio introductorio de ROS 2. Su propósito principal no es enseñar simulación, física ni navegación avanzada, sino entregar a los estudiantes un sistema visual que puedan tratar como una **caja negra ROS 2**.

Los estudiantes deberán posteriormente desarrollar sus propios nodos para interactuar con el simulador mediante tópicos ROS 2.

El sistema debe ser:

- visual;
- simple;
- fácil de entender;
- fácil de modificar;
- suficientemente liviano para ejecutarse en computadores de laboratorio;
- independiente de Gazebo;
- construido en Python;
- compatible con ROS 2 Jazzy.

No es código de producción. Se debe priorizar claridad y simplicidad sobre arquitecturas complejas, patrones de diseño, abstracciones generales o cobertura exhaustiva mediante tests.

---

# 2. Contexto de uso

La actividad docente considera aproximadamente los siguientes conceptos:

- creación de workspace;
- creación de paquetes;
- creación y ejecución de nodos;
- publishers;
- subscribers;
- `ros2 topic list`;
- `ros2 topic echo`;
- `rqt_graph`;
- opcionalmente `ros2 bag`.

El simulador debe permitir que estos conceptos aparezcan de forma natural dentro de una única actividad.

El estudiante NO debe necesitar comprender el código interno del simulador para utilizarlo.

La interfaz pública del simulador debe ser exclusivamente mediante ROS 2.

---

# 3. Requisito de entorno ROS 2

El desarrollo y las pruebas deben realizarse sobre **ROS 2 Jazzy**.

Antes de utilizar cualquier comando `ros2`, ejecutar explícitamente:

```bash
source /opt/ros/jazzy/setup.bash
```

Esto debe considerarse durante todo el desarrollo.

Por ejemplo:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```

Si se crea un workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

No asumir que ROS 2 ya se encuentra cargado en `.bashrc`.

---

# 4. Concepto del simulador

El escenario consiste en una pista 2D con:

- punto de inicio;
- rectas;
- curvas amplias;
- eventualmente cambios moderados de ancho;
- línea de meta.

El vehículo comienza automáticamente en una posición y orientación definidas por el nivel.

Ejemplo conceptual:

```text
START
  ↓
┌───────────────────────────────────┐
│ ╔══════════════════════╗          │
│ ║                      ╚════╗     │
│ ║                           ║     │
│ ╚══════╗                    ║     │
│        ║        ╔═══════════╝     │
│        ╚════════╝          🏁     │
└───────────────────────────────────┘
```

El objetivo eventual del estudiante será programar un nodo capaz de conducir el vehículo desde START hasta GOAL utilizando la información sensorial publicada por el simulador.

---

# 5. Modelo del vehículo

Utilizar un modelo cinemático 2D deliberadamente simple.

Estado:

```text
x
y
yaw
```

Entrada:

```text
linear.x
angular.z
```

provenientes de:

```text
/cmd_vel
geometry_msgs/msg/Twist
```

No implementar:

- Ackermann steering;
- dinámica de neumáticos;
- suspensión;
- torque;
- masa;
- aceleraciones físicas detalladas;
- simulación dinámica avanzada.

El vehículo puede dibujarse visualmente como un automóvil, aunque internamente utilice un modelo tipo unicycle/differential-drive.

Se deben aplicar límites razonables a:

```text
linear.x
angular.z
```

para evitar velocidades absurdas.

---

# 6. Interfaz ROS 2

## 6.1 Subscriber

El simulador debe suscribirse a:

```text
/cmd_vel
```

Tipo:

```text
geometry_msgs/msg/Twist
```

Utilizar:

```text
msg.linear.x
msg.angular.z
```

como velocidad lineal y angular.

---

## 6.2 Pseudo-LiDAR

El simulador debe publicar:

```text
/scan
```

Tipo:

```text
sensor_msgs/msg/LaserScan
```

Debe simular un LiDAR 2D mediante ray casting contra los límites de la pista.

No es necesario que el sensor sea físicamente exacto.

Configuración inicial sugerida:

```text
FOV: 180°
Número de rayos: 61 o 91
range_min: aproximadamente 0.05 m
range_max: aproximadamente 5–10 m
```

La configuración debe estar concentrada en pocas constantes claramente identificables.

Los rayos deben mostrarse opcionalmente sobre la interfaz gráfica para facilitar la explicación docente.

---

## 6.3 Odometry

Publicar:

```text
/odom
```

Tipo:

```text
nav_msgs/msg/Odometry
```

Debe representar la posición simulada:

```text
x
y
yaw
```

No es necesario implementar ruido ni deriva inicialmente.

El objetivo es que los estudiantes puedan ejecutar:

```bash
ros2 topic echo /odom
```

y observar información consistente con el movimiento visual.

---

# 7. Meta

Cada nivel debe contener una línea o región de meta.

Cuando el vehículo llegue a ella:

- mostrar visualmente `GOAL REACHED`;
- detener opcionalmente el vehículo;
- mantener visible el resultado;
- publicar opcionalmente un tópico:

```text
/goal_reached
std_msgs/msg/Bool
```

Este tópico es deseable, pero no crítico para la primera versión.

---

# 8. Colisiones / salida de pista

El vehículo no debe atravesar libremente los límites de la pista.

Implementar una detección simple.

Al colisionar o abandonar la pista:

- detener el vehículo;
- mostrar claramente `COLLISION` o `OUT OF TRACK`;
- permitir reiniciar el nivel.

No desarrollar un motor físico de colisiones.

Una comprobación geométrica simple es suficiente.

---

# 9. Definición editable de niveles

Este es un requisito importante.

Los niveles NO deben estar hardcodeados en Python.

Deben cargarse desde archivos externos fáciles de editar manualmente.

Preferir un formato simple como:

```text
CSV
```

o:

```text
TXT
```

No utilizar formatos innecesariamente complejos.

## Formato recomendado

Se recomienda representar el centro de la pista mediante una secuencia ordenada de puntos:

```csv
x,y,width
0.0,0.0,2.0
2.0,0.0,2.0
4.0,0.5,2.0
6.0,2.0,2.0
7.0,4.0,2.0
8.5,5.0,2.0
10.0,5.0,2.0
```

El simulador debe construir visualmente la pista a partir de estos puntos.

Puede utilizar interpolación entre puntos para generar curvas suaves.

La prioridad es que un docente pueda crear un nuevo nivel modificando un archivo de texto sin tocar código Python.

Evitar requerir herramientas gráficas externas para crear un nivel.

---

# 10. Metadata del nivel

Cada nivel necesita además definir:

```text
posición inicial x
posición inicial y
yaw inicial
posición/región de meta
```

Se puede resolver mediante un archivo complementario simple.

Ejemplo:

```txt
start_x=0.5
start_y=0.0
start_yaw=0.0

goal_x=10.0
goal_y=5.0
goal_radius=0.5
```

Alternativamente puede integrarse en el mismo archivo si la solución sigue siendo fácil de leer.

No diseñar un parser genérico complejo.

---

# 11. Selección del nivel al ejecutar

El nivel debe seleccionarse al lanzar el simulador.

La interfaz deseada es similar a:

```bash
ros2 launch track_sim simulator.launch.py level:=level1
```

o:

```bash
ros2 run track_sim simulator --ros-args -p level:=level1
```

Idealmente soportar ambas si hacerlo no agrega demasiada complejidad.

Los niveles deberían encontrarse, por ejemplo, en:

```text
track_sim/
├── levels/
│   ├── level1.csv
│   ├── level1.txt
│   ├── level2.csv
│   ├── level2.txt
│   └── level3.csv
```

La prioridad es que agregar:

```text
level4
```

no requiera modificar el código del simulador.

---

# 12. Niveles iniciales

Crear al menos tres escenarios.

## Level 1 — Basic

- recta inicial;
- una curva amplia;
- recta final;
- pista relativamente ancha.

Debe ser muy fácil de recorrer.

## Level 2 — Curves

- al menos dos curvas amplias;
- curvas en sentidos opuestos;
- rectas intermedias;
- dificultad moderada.

Este será probablemente el escenario principal del laboratorio.

## Level 3 — Narrow

- recorrido similar;
- algunas zonas algo más estrechas;
- curvas algo más pronunciadas.

No debe requerir algoritmos de navegación avanzados.

---

# 13. Interfaz gráfica

Utilizar una biblioteca Python liviana.

Preferentemente:

```text
pygame
```

La ventana debe mostrar como mínimo:

- pista;
- vehículo;
- punto de inicio;
- meta;
- estado actual.

Opcionalmente:

- rayos del LiDAR;
- trayectoria recorrida;
- velocidad lineal;
- velocidad angular;
- tiempo transcurrido.

La visualización debe privilegiar claridad sobre estética.

No incorporar frameworks gráficos pesados.

---

# 14. Escala visual

Implementar una conversión sencilla:

```text
metros del mundo → píxeles
```

Mantener esta lógica en una función claramente identificable.

Evitar sistemas generales de escenas, cámaras o motores gráficos.

Si es necesario agregar cámara, utilizar una implementación mínima.

Idealmente los niveles iniciales completos deben caber dentro de una ventana fija.

---

# 15. Controles manuales

Agregar controles de teclado únicamente como herramienta de prueba del simulador.

Por ejemplo:

```text
W / ↑ : avanzar
S / ↓ : retroceder
A / ← : girar izquierda
D / → : girar derecha
R     : reset
ESC   : salir
```

El control manual NO debe reemplazar `/cmd_vel`.

Debe existir principalmente para comprobar rápidamente los niveles y sensores durante el desarrollo.

Si existe simultáneamente un nodo publicando `/cmd_vel`, evitar comportamientos confusos.

Puede implementarse un parámetro:

```text
keyboard_control:=true/false
```

Default sugerido:

```text
false
```

---

# 16. Reset

Debe existir una forma sencilla de reiniciar el nivel.

Mínimo:

```text
tecla R
```

Deseable:

```text
/reset
std_srvs/srv/Empty
```

El servicio es opcional en primera versión si aumenta innecesariamente el alcance.

---

# 17. Arquitectura del paquete

Mantener una estructura pequeña.

Ejemplo deseado:

```text
track_sim/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
├── launch/
│   └── simulator.launch.py
├── levels/
│   ├── level1.csv
│   ├── level1.txt
│   ├── level2.csv
│   └── level2.txt
└── track_sim/
    ├── __init__.py
    ├── simulator.py
    ├── track.py
    └── level_loader.py
```

No crear más módulos salvo que exista una razón clara.

El código debería permitir a un estudiante con Python básico seguir el flujo:

```text
cargar nivel
→ iniciar ROS
→ recibir cmd_vel
→ actualizar vehículo
→ calcular LiDAR
→ publicar ROS
→ dibujar pantalla
```

---

# 18. Restricciones de diseño de software

Este proyecto tiene finalidad docente.

Priorizar explícitamente:

```text
claridad > generalización
simplicidad > extensibilidad
legibilidad > arquitectura
```

Evitar:

- dependency injection;
- factories;
- interfaces innecesarias;
- múltiples niveles de herencia;
- dataclasses complejas cuando un diccionario o clase sencilla basta;
- patrones de diseño innecesarios;
- configuración excesivamente parametrizada;
- arquitectura basada en plugins;
- frameworks adicionales;
- optimización prematura;
- abstraer una operación que solamente se utiliza una vez.

Se aceptan funciones relativamente directas si son fáciles de comprender.

Agregar comentarios donde expliquen decisiones no obvias, pero evitar comentar cada línea.

---

# 19. Tests

No desarrollar una suite extensa de tests.

Este NO es código de producción.

Como máximo incluir verificaciones simples para:

- carga correcta de un nivel;
- arranque del nodo;
- publicación básica de `/scan`.

No invertir esfuerzo significativo en mocks, fixtures o cobertura.

La validación principal será funcional y visual.

---

# 20. Dependencias

Mantener dependencias al mínimo.

Esperadas:

```text
rclpy
geometry_msgs
sensor_msgs
nav_msgs
std_msgs
pygame
numpy
```

Si una dependencia adicional puede evitarse fácilmente, evitarla.

En particular, no incorporar motores de física o geometría pesados.

El ray casting puede implementarse directamente usando geometría 2D sencilla.

---

# 21. Comportamiento esperado

Después de compilar:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

se debe poder ejecutar:

```bash
ros2 launch track_sim simulator.launch.py level:=level2
```

Debe abrirse inmediatamente la ventana del simulador.

En otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic list
```

debería mostrar al menos:

```text
/cmd_vel
/scan
/odom
```

Ejecutando:

```bash
ros2 topic echo /scan
```

deben observarse las mediciones del LiDAR.

Ejecutando:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...
```

debe poder comprobarse movimiento del vehículo.

---

# 22. Compatibilidad con rqt_graph

La arquitectura debe ser deliberadamente sencilla para que posteriormente pueda visualizarse claramente mediante:

```bash
rqt_graph
```

Una vez que un estudiante implemente su controlador, se espera conceptualmente:

```text
                  /scan
┌───────────────┐ ───────────────► ┌────────────────────┐
│   track_sim   │                  │ student_controller │
└───────────────┘ ◄─────────────── └────────────────────┘
                   /cmd_vel

       │
       └──────── /odom
```

Evitar generar numerosos nodos internos o tópicos auxiliares que ensucien innecesariamente el grafo.

Preferentemente el simulador completo debe ejecutarse como **un único nodo ROS 2**.

---

# 23. Compatibilidad con rosbag

Los siguientes tópicos deben poder registrarse directamente:

```bash
ros2 bag record /scan /odom /cmd_vel
```

No se requiere funcionalidad especial del simulador para rosbag.

Los mensajes deben utilizar timestamps ROS consistentes.

---

# 24. README

Crear un README breve que incluya exclusivamente lo necesario para:

1. instalar dependencias;
2. compilar;
3. ejecutar;
4. seleccionar nivel;
5. probar `/cmd_vel`;
6. observar `/scan`;
7. crear un nuevo nivel.

Debe destacar explícitamente:

```bash
source /opt/ros/jazzy/setup.bash
```

antes de cualquier uso de ROS 2.

Incluir un pequeño ejemplo del formato de niveles.

---

# 25. Criterios de aceptación

La primera versión estará completa cuando:

- [ ] compile correctamente con ROS 2 Jazzy;
- [ ] pueda ejecutarse mediante `ros2 launch`;
- [ ] permita seleccionar distintos niveles;
- [ ] los niveles sean archivos externos editables;
- [ ] agregar un nivel no requiera modificar Python;
- [ ] muestre una pista 2D;
- [ ] muestre un vehículo móvil;
- [ ] reciba `/cmd_vel`;
- [ ] publique `/scan`;
- [ ] publique `/odom`;
- [ ] el pseudo-LiDAR detecte correctamente los bordes de la pista;
- [ ] detecte llegada a la meta;
- [ ] detecte salida de pista/colisión;
- [ ] permita reiniciar fácilmente;
- [ ] incluya al menos tres niveles;
- [ ] pueda controlarse manualmente para pruebas;
- [ ] el código resulte sencillo de leer;
- [ ] no contenga abstracciones innecesarias;
- [ ] no incluya una infraestructura de tests desproporcionada;
- [ ] incluya un README breve y funcional.

---

# 26. Prioridad de implementación

Implementar en este orden:

1. paquete ROS 2 funcional;
2. carga de nivel desde archivo;
3. representación visual de pista;
4. vehículo y `/cmd_vel`;
5. colisiones/salida de pista;
6. `/odom`;
7. pseudo-LiDAR y `/scan`;
8. meta;
9. selección de nivel mediante launch;
10. niveles de ejemplo;
11. controles manuales;
12. README.

No agregar características fuera de este alcance antes de que estos puntos funcionen.

---

# 27. Fuera de alcance

No implementar:

- Gazebo;
- URDF;
- TF complejo;
- Nav2;
- SLAM;
- mapas occupancy grid;
- planificación de trayectorias;
- control autónomo;
- PID del vehículo;
- comportamiento para completar la pista;
- sensores de cámara;
- dinámica vehicular realista;
- networking;
- interfaz web;
- editor gráfico de niveles;
- sistema de plugins;
- benchmarks;
- CI/CD complejo;
- Docker salvo necesidad explícita.

El **controlador autónomo debe quedar deliberadamente fuera del simulador**, ya que será desarrollado por los estudiantes.