# Entorno Figure Follower (Español)
Este es un entorno diseñado para que sea resuelto por un agente que controla un robot omnidireccional utilizando imagenes de la cámara como observaciones. El robot cuenta con cuatro ruedas mecanum, cada una de ellas accionadas por un motor independiente. El agente debe aprender a moverse a través de un camino donde cada señal que encuentre le indicará a dónde ir a continuación. Cada una de estas señales tiene una forma geométrica. Los movimientos que el agente debe aprender a hacer cada vez que ve una de las señales son:
* Triangulo: girar a la izquierda.
* Cuadrado: girar a la derecha.
* Pentagono: dar media vuelta.
* Círculo: parar.

Conforme el robot se acerque a la siguiente señal en el camin, el agente recibirá un poco de recompensa, para ser precisos, es un punto por cada metro. Si el robot colisiona recibe una penalización de 10 puntos. Tambi´n el agente pierde algo de recompensa al activar sus motores.

## Uso
Hay dos maneras de utilizar este entorno. Con la primera es necesario importar el paquete de ```gymnasiun``` así como este y posteriormente utilizar el tradicional ```gymnasium.make()``` para crear e inicializar el entorno, como se muestra en el siguiente fragmento de código.

```python
import gymnasium
import FigFollowerEnv

env = gym.make('FigFollowerEnv-v1', render_mode="rgb_array")
```

Aunque también puedes crear el entorno de esta manera.

```python
import gymnasium
from FigFollowerEnv import FigFollowerV1

env = FigFollowerV1()
```

Hay unas cuantas opciones para personalizar el entorno.
Estas son todas ellas.
* width : int
  - El ancho del campo de visión del robot.
* height : int
  - El alto del campo de visión del robot.
* fps : int
  - Cuantas veces se actualiza por segundo el ambiente. Útil para simular lag
* max_speed : float
  - Maxima velocidad de los motores.
* max_time : int
  - Tiempo total (en segundos de la simulación) que durará el episodio.
* nodes : int
  - Cuantas señales poner en el camino.
* render_mode : str
  - Indica cómo renderizar el ambiente a Gymnasium.

## Instalación
Debido a que este paquete aún no está completado del todo, no lo subí a PyPI y también me dio flojera así que por ahora hay que instalarlo manualmente. Para hacerlo ve a la caperta del proyecto y corre el siguiente comando.

```pip install .```

Si no quieres instalarlo puedes copiar la carpeta FigFollowerEnv a tus proyectos.

## Extra
Si necesitas utilizar el URDF y XACRO del robot omnidireccional y de los demás assets adelante. Fue bastante pesado hacer esos modelos y si puedo ayudar a alguien a alivianar un pco su carga estaré feliz de haber ayudado.

## Agradecimientos
El código en este repositorio fue resultado de un proyecto en el que participé durante mi estancia en el IPICyT
Quiero agradecer al IPICyT y CONAHCYT por haberme permitido participar en este proyecto así como por el apoyo que me brindaron. También quiero agradecer al Dr. Juan Gonzalo Barajas Ramírez, a mis compañeros del laboratorio y personal de la institución por todo.
¡Muchas gracias!

Esto lo pongo de mi propia voluntad, simplemente me nació.

# Figure Follower Environment (English)

This is a simple environment designed to be resolved by an agent embodied in an omnidirectional robot using camera images as observations from the environment. The robot has four mecanum wheels actioned through independent motors. The agent must learn how to move through a path where each signal it encounters tells it where to go next. Each one of these signals have the form of geometric figures. The movements that the agent should learn to do once it sees the signals are the next:
* Triangle: go left.
* Square: go right.
* Pentagon: turn around.
* Circle: stop.

As the robot approaches the coming signal in the path the agent receives some reward, to be precise, it's one point of reward per meter. If the robot crashes with the signals the agent receives a penalization of 10 points. Also the sgent loses some reward for activating its motors.

## Usage
There are two ways to use this environment. The first one involves importing the ```gymnasiun``` package as well as this one and then using the traditional ```gymnasium.make()``` function to create and initialize the environment as shown in the following code snippet.


```python
import gymnasium
import FigFollowerEnv

env = gym.make('FigFollowerEnv-v1', render_mode="rgb_array")
```

But you can also create the environment this way.

```python
import gymnasium
from FigFollowerEnv import FigFollowerV1

env = FigFollowerV1()
```

There a couple of options you can use to customize this environment. These are all of them:
* width : int
  - The width of the robot's camera field of view.
* height : int
  - The height of the robot's camera field of view.
* fps : int
  - The refresh rate at which the camera sends images. Useful for simulating lag.
* max_speed : float
  - Maximum speed for the wheels to spin.
* max_time : int
  - Maximum number of simulation seconds allowed.
* nodes : int
  - How many signals are in the path.
* render_mode : str
  - Indicates Gymansium how to render this environment.

## Installation
Due to this package not being finished as of now I didn't upload it to PyPi, also I felt the laziness creeping up my back so yeah, for now we will have to install it manually, to do that go to the project's folder and use this command.

```pip install .```

If you don't want to install the package you can also copy the FigFollowerEnv into your project and use it normally.

## Extra
If you need to use the URDF and XACRO files for the four-wheeled omnidirectional robot and the other assets, go ahead, it's in the assets folder. It was incredibly cumbersome making those models and if I can leverage some of that work to someone else I'm happy to have been of help.

## Thanks
The code on this repo was a result of a project I was part of during my internship at the IPICyT.
I want to thank the IPICyT and CONAHCYT for allowing me to participate in this project as well for all the support they provided.
Also I want to thank Phd. Juan Gonzalo Barajas Ramírez, my lab mates and all of the institutions' personnel for everything.
Thank You!

I write this out of my own volition, it's simply something that I felt I needed to do.
