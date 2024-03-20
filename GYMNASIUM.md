Autor: Javier Gómez Eguizábal

Fecha: 19/03/24

## DIMENSIONES DEL MAPA

```bash
(J A V I E R = 6 * 2 = 12) -> max_x = 12 

(G O M E Z = 5 * 2 = 10) -> max_y = 10
```
----

## MAPA DISEÑADO
### [MAPA 1: map1](map1.csv)

<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/30e3d431-ae61-4059-8aa3-992286c742c5" width = 30%/>
</p>

----

## CODIGO CREADO
### CODIGO 1: gym.py
Este código emplea la biblioteca Gym para crear y controlar un entorno de simulación. Este entorno está basado en un mapa donde el cuadrado representado o agente es capaz de moverse en direcciones diferentes dentro de él.

- Se ha aumentado el tiempo de espera entre iteraciones para poder visualizar mejor el recorrido
- El codigo imprime por pantalla los datos mas relevantes
- Al llegar a meta, lo imprime por pantalla y cierra la ventana del laberinto

```bash
#!/usr/bin/env python

#Importar bibiliotecas necesarias
import gymnasium as gym
import gymnasium_csv
import numpy as np
import time

#Definir constantes y configuraciones iniciales
UP = 0
UP_RIGHT = 1
RIGHT = 2
DOWN_RIGHT = 3
DOWN = 4
DOWN_LEFT = 5
LEFT = 6
UP_LEFT = 7
SIM_PERIOD_MS = 500.0
CONT=0

#Crear entorno de gym para poder trabajar con el mapa deseado, seleccionando origen y meta
env = gym.make('gymnasium_csv-v0',
               render_mode='human',  # "human", "text", None
               inFileStr='../assets/map1.csv',
               initX=2,
               initY=2,
               goalX=8,
               goalY=10)
observation, info = env.reset()
print("observation: "+str(observation)+", info: "+str(info))
env.render()
time.sleep(0.5)

#Bucles de control que guian hasta la meta
if CONT==0:
    for i in range(6):
        #Mover hacia abajo a la derecha
        observation, reward, terminated, truncated, info = env.step(DOWN_RIGHT)
        #Renderizar el entorno
        env.render()
        #Imprimir informacion relevante
        print("observation: " + str(observation)+", reward: " + str(reward) + ", terminated: " +
            str(terminated) + ", truncated: " + str(truncated) + ", info: " + str(info))
        #Esperar 1s
        time.sleep(SIM_PERIOD_MS/500.0)
        #print(i,CONT) //debug
        #Aumentar en 1 el contador si ya se han realizado los movimientos deseados
        if i==5:
            CONT=1


if CONT==1:
    for j in range(2):
        #Mover hacia la derecha
        observation, reward, terminated, truncated, info = env.step(RIGHT)
        env.render()
        print("observation: " + str(observation)+", reward: " + str(reward) + ", terminated: " +
            str(terminated) + ", truncated: " + str(truncated) + ", info: " + str(info))
        time.sleep(SIM_PERIOD_MS/500.0)
        #print(j) //debug
        if j==1:
            CONT=2

#Fin del recorrido
if CONT==2:
    print("HA LLEGADO A LA POSICION OBJETIVO")
```

### CODIGO 2: gym2.py

```bash
#BFS

import gymnasium as gym
import gymnasium_csv
import numpy as np
import time

# Constantes y configuraciones iniciales
UP = 0
UP_RIGHT = 1
RIGHT = 2
DOWN_RIGHT = 3
DOWN = 4
DOWN_LEFT = 5
LEFT = 6
UP_LEFT = 7
SIM_PERIOD_MS = 500.0

# Crear entorno de gym
env = gym.make('gymnasium_csv-v0',
               render_mode='human',  # "human", "text", None
               inFileStr='../assets/map2.csv',
               initX=2,
               initY=2,
               goalX=8,
               goalY=10)

# Función para encontrar el camino hacia la meta utilizando búsqueda en anchura
def find_path(env):
    start_state = env.reset()
    queue = [(start_state, [])]

    while queue:
        state, path = queue.pop(0)
        if env.is_goal(state):
            return path
        for action in range(env.action_space.n):
            next_state, _, _, _ = env.step(action)
            if next_state is not None:
                queue.append((next_state, path + [action]))
                env.undo_step()

# Encontrar el camino hacia la meta
path = find_path(env)

# Seguir el camino hacia la meta
for action in path:
    observation, reward, terminated, truncated, info = env.step(action)
    env.render()
    time.sleep(SIM_PERIOD_MS / 500.0)

print("HA LLEGADO A LA POSICION OBJETIVO")

```

## VIDEOS

### gym.py & map1

https://youtu.be/GJMQQUQM4DQ

### gym2.py & map2


