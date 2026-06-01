# 🤖 Tutorial: Cómo Hacer que tu TurtleBot4 No te Odie (y Viceversa)

*Asumiendo que ya estás en la red. Si no lo estás, no lo estás.*

---

## Paso 1: Entrar al robot como Dios manda

Abre una terminal en tu laptop (sí, directamente, sin VM, como una persona normal) y conéctate por SSH:

```bash
ssh ubuntu@10.42.0.1
# contraseña: turtlebot4
# (sí, la contraseña del robot de $1,500 USD es "turtlebot4". Súper Fort Knox.)
```

---

## Paso 2: El ritual del ROS_DOMAIN_ID

ROS usa un número del 0 al 255 para que los robots no se hablen entre sí y armen el caos. Tú debes usar el mismo número en el robot y en tu laptop, o si no, estarás hablando al vacío como cuando mandas un mensaje y dejan en visto.

```bash
# Ver qué dominio tienes
echo $ROS_DOMAIN_ID

# Si no te gusta el número, cámbialo
export ROS_DOMAIN_ID=21   # usa el número que te toque

# Hazlo también en tu laptop. Los dos. El mismo. Sí, los dos.
```

Y siempre después de esto:

```bash
source /opt/ros/jazzy/setup.bash
```

*(Si no haces esto, ROS te mirará con decepción.)*

---

## Paso 3: Verificar que el robot y tu laptop se están escuchando

Este es el momento de la verdad. Abre **dos terminales**: una con SSH al robot, otra normal en tu laptop.

**En el robot** (tu terminal SSH):
```bash
ros2 run demo_nodes_cpp listener
```

**En tu laptop** (otra terminal, sin SSH):
```bash
ros2 run demo_nodes_cpp talker
```

Si ves mensajes pasando de uno al otro: 🎉 eres un genio.

Si no ves nada: llama a **Cortijo**. *(El README lo dice literalmente. Dos veces. Es la solución oficial.)*

---

## Paso 4: Instalar los sensores y despertar al robot

En el robot, instala todo lo necesario para que el TurtleBot sea algo más que un Roomba con ego:

```bash
sudo apt install -y \
  ros-jazzy-rplidar-ros \
  ros-jazzy-depthai-ros \
  ros-jazzy-irobot-create-nodes \
  ros-jazzy-turtlebot4-msgs \
  ros-jazzy-turtlebot4-description \
  ros-jazzy-turtlebot4-bringup
```

Luego **apaga y prende el robot**. Sí, el clásico "¿lo apagaste y lo volviste a prender?".

---

## Paso 5: El Bringup — la misa de ROS

Reconéctate por SSH y ejecuta:

```bash
ros2 launch turtlebot4_bringup lite.launch.py
```

Sabrás que funcionó cuando el robot haga el sonido: **"pu puru pupu 🎵"**

En serio. Eso dice el README. Ese es el indicador oficial de éxito. Un robot de investigación cuya métrica de funcionamiento es "¿hizo el sonido chévere?".

Luego en otra terminal:
```bash
ros2 topic list
```

Deberías ver una lista larga que incluya `/scan`, `/cmd_vel`, `/odom`, `/oakd/rgb/preview/image_raw`, entre otros. Si no aparecen todos... ya sabes. **Cortijo.**

---

## Paso 6: Mover el robot (el momento que esperabas)

**En el robot** — para ver qué comandos llegan:
```bash
ros2 topic echo /cmd_vel
```

**En tu laptop** — para controlar el robot con el teclado:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Usa `W/A/S/D` para mover. Trata de no chocar el robot con una pared el primer día. O sí. Aprendes más así.

---

## Paso 7: Ver la cámara y el LiDAR (opcional pero satisfactorio)

```bash
# Cámara OAK-D
ros2 launch turtlebot4_bringup oakd.launch.py

# LiDAR RPLidar
ros2 launch turtlebot4_bringup rplidar.launch.py

# Ver imagen en tiempo real
ros2 run rqt_image_view rqt_image_view
```

---

## 🆘 Árbol de decisión oficial del laboratorio

```
¿Funciona?
├── Sí → 🎉 Felicidades
└── No → ¿Lo apagaste y lo prendiste?
         ├── No → Hazlo
         └── Sí → ¿Revisaste el ROS_DOMAIN_ID?
                  ├── No → Revísalo
                  └── Sí → Llama a Cortijo
```

---

> **Nota al margen:** La contraseña del WiFi del lab y otros secretos están hardcodeados en el README público de GitHub. La ciberseguridad es un viaje, no un destino. 🔐
