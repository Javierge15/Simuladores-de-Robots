Autor: Javier Gómez Eguizábal
Fecha: 21/12/24

## DIMENSIONES DEL MAPA

```bash
(J A V I E R = 6 * 2 = 12) -> max_x = 12 

(G O M E Z = 5 * 2 = 10) -> max_y = 10
```

## MAPAS CREADOS
### MapaSimple
<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/d7b4f36b-e060-4776-a0b3-9a3baac83443" width = 30%/>
</p>

### map
<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/b92f0064-eb1e-4734-abc8-5dfc1a0dba65" width = 30%/>
</p>

### map2
<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/a80dbe6e-127a-4edb-a78a-da2a964e8f9a" width = 30%/>
</p>

### Map3
<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/4fbc6e18-dc0e-4bfa-a49c-41d6e3d9ee03" width = 30%/>
</p>

### Map4
<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/3afd1475-3ad3-4271-ac42-fc3de05f14b2" width = 30%/>
</p>

### Map5
<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/098912c6-4db5-49c9-bb80-d6ea12845b99" width = 30%/>
</p>

## CONTROLADORES IMPLEMENTADOS
Se han diseñado dos controladores diferentes, ambos basados en el seguimiento de paredes.
### Sigue_Paredes_Derecha

```bash
## Sigue Paredes Derecha

## Realizado por: Javier Gómez Eguizábal
## Fecha: 18-02-2024

# Import some classes of the controller module
from controller import Robot, Motor, DistanceSensor,GPS

# Create the Robot instance
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Device configuration
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
ds_frontal = robot.getDevice('distance_sensor_frontal')
ds_derecha = robot.getDevice('distance_sensor_derecha')
ds_izquierda = robot.getDevice('distance_sensor_izquierda')
gps = robot.getDevice('gps')
imu = robot.getDevice('inertial unit')

# Enable sensors
ds_frontal.enable(timestep)
ds_derecha.enable(timestep)
ds_izquierda.enable(timestep)
gps.enable(timestep)
imu.enable(timestep)

#Set initial positions and velocities for motors
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# Variable inicialization
cont=0

# Main Loop
while robot.step(timestep) != -1:
  # Read the sensors:
  dist_izq = ds_izquierda.getValue()
  dist_der = ds_derecha.getValue()
  dist_front = ds_frontal.getValue()

  imu_rads = imu.getRollPitchYaw()
  yaw = imu_rads[2]

  gps_vals = gps.getValues()
  x = gps_vals[0]
  y = gps_vals[1] 

  # Colocar al robot en la posicion inicial, acercándolo a la pared
  if cont == 0:
  #Hasta que no se encuentre colocado a 90º, no comienza a moverse
    if yaw <= -1.5708:
       cont = 1
       motor_left.setVelocity(5.0)
       motor_right.setVelocity(5.0)
       print
    else:
       motor_left.setVelocity(1.0)
       motor_right.setVelocity(-1.0)

  #Una vez se encuentre pegado a la pared, entra en juego el sigueparedes
  if cont == 1 and dist_front <= 465:
     cont = 3

  #Codigo del sigueparedes
  if cont == 3:
    #Si detecta un objeto delante, gira de manera brusca hacia la izquierda
    if dist_front < 999:
      print('Gira en el sitio')
      motor_left.setVelocity(-5.0)
      motor_right.setVelocity(5.0)
        
    else:
      #Si detecta algo a la derecha, se mueve hacia delante, siguiendo la pared
      if dist_der < 999:
        print('Hacia delante')
        motor_left.setVelocity(5.0)
        motor_right.setVelocity(5.0)  
        
      #En caso de no detectar nada a la derecha, gira hacia este lado intentando recuperar la pared
      else:
        print('Gira Derecha')
        motor_left.setVelocity(5.0)
        motor_right.setVelocity(1.0)  

  # Verify if the robot reached the finish line 
  if x >= 8.5 and y >= 10.5:
    break

  #Print imfnormation
  print('Der:', dist_der, 'Cent:', dist_front, 'Izq:', dist_izq, 'X:', x, 'Y:', y, 'Yaw:', yaw, 'CONT',cont)

# Enter here exit cleanup code.
# Detener los motores antes de salir
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)    
print('META ACANZADA')

```
### Sigue paredes izquierda

```bash
## Sigue Paredes Izquierda

## Realizado por: Javier Gómez Eguizábal
## Fecha: 18-02-2024

# Import some classes of the controller module
from controller import Robot, Motor, DistanceSensor,GPS

# Create the Robot instance
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Device configuration
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
ds_frontal = robot.getDevice('distance_sensor_frontal')
ds_derecha = robot.getDevice('distance_sensor_derecha')
ds_izquierda = robot.getDevice('distance_sensor_izquierda')
gps = robot.getDevice('gps')
imu = robot.getDevice('inertial unit')

# Enable sensors
ds_frontal.enable(timestep)
ds_derecha.enable(timestep)
ds_izquierda.enable(timestep)
gps.enable(timestep)
imu.enable(timestep)

#Set initial positions and velocities for motors
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# Variable inicialization
cont=0

# Main Loop
while robot.step(timestep) != -1:
  # Read the sensors:
  dist_izq = ds_izquierda.getValue()
  dist_der = ds_derecha.getValue()
  dist_front = ds_frontal.getValue()

  imu_rads = imu.getRollPitchYaw()
  yaw = imu_rads[2]

  gps_vals = gps.getValues()
  x = gps_vals[0]
  y = gps_vals[1] 

  # Colocar al robot en la posicion inicial, acercándolo a la pared
  if cont == 0:
  #Hasta que no se encuentre colocado a 90º, no comienza a moverse
    if yaw <= -1.5708:
       cont = 1
       motor_left.setVelocity(5.0)
       motor_right.setVelocity(5.0)
       print
    else:
       motor_left.setVelocity(1.0)
       motor_right.setVelocity(-1.0)

  #Una vez se encuentre pegado a la pared, entra en juego el sigueparedes
  if cont == 1 and dist_front <= 465:
     cont = 3

  #Codigo del sigueparedes
  if cont == 3:
    #Si detecta un objeto delante, gira de manera brusca hacia la derecha
    if dist_front < 999:
      print('Gira en el sitio')
      motor_left.setVelocity(5.0)
      motor_right.setVelocity(-5.0)
        
    else:
      #Si detecta algo a la izquierda, se mueve hacia delante, siguiendo la pared
      if dist_izq < 999:
        print('Hacia delante')
        motor_left.setVelocity(5.0)
        motor_right.setVelocity(5.0)  
        
      #En caso de no detectar nada a la izquierda, gira hacia este lado intentando recuperar la pared
      else:
        print('Gira Derecha')
        motor_left.setVelocity(1.0)
        motor_right.setVelocity(5.0)  

  # Verify if the robot reached the finish line 
  if x >= 8.5 and y >= 10.5:
    break

  #Print imfnormation
  print('Der:', dist_der, 'Cent:', dist_front, 'Izq:', dist_izq, 'X:', x, 'Y:', y, 'Yaw:', yaw, 'CONT',cont)

# Enter here exit cleanup code.
# Detener los motores antes de salir
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)    
print('META ACANZADA')
```

## Videos

### Controlador 1: https://youtu.be/wivzPyxKoQI?si=ecKB0m-OpVU4D8Z6

  -Empezando fuera de la pared: https://www.youtube.com/watch?v=Q5aCFpVGufg
  
### Controlador 2: https://youtu.be/p29dAyOZ4fA?si=zeML4aBzF0gGPu2L


