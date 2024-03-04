Autor: Javier Gómez Eguizábal

Fecha: 03/03/24

## DIMENSIONES DEL MAPA

```bash
(J A V I E R = 6 * 2 = 12) -> max_x = 12 

(G O M E Z = 5 * 2 = 10) -> max_y = 10
```

## MAPAS CREADOS
### [MAPA 1: map.world](map.world.xml)
<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/b5b23037-ea4d-4707-8bff-b934f2c821ae" width = 30%/>
</p>

<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/91c935d2-e23d-4c95-b3cf-cad2dfdfc6d4" width = 60%/>
</p>

### [MAPA 2: map.world2](map.world2.xml)
<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/8ae9b5cb-c72f-4e9e-a532-facbd9737a60" width = 30%/>
</p>

<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/3945b31a-bdd9-43b8-885c-24692ce8bd51" width = 60%/>
</p>

## PLUGINS CREADOS

### Plugin 1:
Se ha diseñado un plugin de Gazebo escrito en C++. Este plugin, llamado "MyWheels", se encarga de controlar un robot pioneer en el entorno de simulación de Gazebo. El código implementa el control del movimiento de las ruedas para que el modelo siga una ruta predefinida en el espacio tridimensional y sea capaz de llegar a la meta.

```bash
// Inclusion de bibliotecas
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

//Inicia un espacio llamad gazebo
namespace gazebo
{
    //Crea una clase llamada MyWheels
    class MyWheels : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            //Almacena el puntero al modelo
            model = _model;

            if (!_sdf->HasElement("left_joint"))
                gzerr << "MyWheels plugin missing <left_joint> element\n";

            if (!_sdf->HasElement("right_joint"))
                gzerr << "MyWheels plugin missing <right_joint> element\n";

            leftJoint = _model->GetJoint(_sdf->GetElement("left_joint")->Get<std::string>());
            rightJoint = _model->GetJoint(_sdf->GetElement("right_joint")->Get<std::string>());

            if (!leftJoint)
                gzerr << "Unable to find left joint[" << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
            if (!rightJoint)
                gzerr << "Unable to find right joint[" << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";

            //Conecta la funcion OnUpdate al evento de actualizacion del mundo
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MyWheels::OnUpdate, this));
        }

        //Declara a inicializa la variable cont
        int cont=0;

        //Define la funcion OnUpdate que se llamará en cada iteracion de la simulacion
        void OnUpdate()
        {
            //Obtiene la Posicion y Orientacion
            ignition::math::Pose3d pose = model->WorldPose();

            //Las imprime por pantalla
            printf("Posicion actual: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
            printf("Orientacion actual: %f %f %f\n", pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());

            //Establece una velocidad constante para el modelo
            double speed = 5.0;

            //Si el contador es 0, inicia la marcha el robot
            if (cont==0)
            {
                leftJoint->SetVelocity(0, speed);
                rightJoint->SetVelocity(0, speed);
            }   

            //En caso de alcanzar el limite inferior, aumenta el valor del contador a 1
            if (cont==0 && pose.Pos().X()>8 && pose.Pos().Y()<=10)
            {
                cont=1;
            }   

            //Rota alrededor de si mismo
            if (cont==1)
            {
                leftJoint->SetVelocity(0, 0.0);
                rightJoint->SetVelocity(0, 0.5);
            }

            //Una vez alcanzada la orientacion deseada, aumenta el valor del contador a 2
            if (cont=1 && pose.Rot().Yaw()>=3.14/2)
            {
                cont=2;
            }

            //En caso de estar en el valor 2, vuelve a emprender la marcha
            if (cont==2)
            {
                leftJoint->SetVelocity(0, speed);
                rightJoint->SetVelocity(0, speed);
            }   
            
            //Imprime el valor del contador para debugging
            printf("cont: %d\n", cont);

            // Una vez llega a la meta, deja de moverse e imprime FIN DEL RECORRIDO
            if (pose.Pos().X()>=8 && pose.Pos().Y()>=10.5)
            {
                leftJoint->SetVelocity(0, 0);
                rightJoint->SetVelocity(0, 0);
                printf("\n\nFIN DEL RECORRIDO\n\n");
                cont=3;
            }        

        }

    private:
        physics::ModelPtr model; 
        physics::JointPtr leftJoint, rightJoint;
        event::ConnectionPtr updateConnection; 
    };

    //Registra el plugin al simulador
    GZ_REGISTER_MODEL_PLUGIN(MyWheels)
}
```
## [VIDEO-PLUGIN 1 map.world](https://youtu.be/4jBIY4Zbtvw)


### Plugin 2:
Este segundo plugin utiliza un sensor láser para detectar obstáculos y ajusta la velocidad de las ruedas en función de la detección. El robot gira a la izquierda si hay obstáculos frente a él o muy cerca de la pared, avanza si no hay obstáculos de frente y detecta la pared y gira a la derecha si no la detecta. El movimiento se detiene cuando el robot alcanza la meta especificada, y se imprime "FIN DEL RECORRIDO".

```bash
// Inclusion de bibliotecas
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/RaySensor.hh>

//Inicia un espacio llamad gazebo
namespace gazebo
{
    //Crea una clase llamada MyWheels
    class MyWheels : public ModelPlugin
    {
    public:
        //FuncionLoad que se llama cuando se carga el plugin
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            int sensor_num = _model->GetSensorCount();
            sensor = sensors::get_sensor("my_sensor");
            if (!sensor)
            {
                gzerr << "the plugin requires a LaserSensor.\n";
                return;
            }
            sensor->SetActive(true);
            gzdbg << "Opened " << sensor->ScopedName() << "\n";
            raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
            if (!raySensor)
            {
                gzerr << "dynamic_pointer_cast to RaySensor failed!\n";
                return;
            }
            // this block of parameters is obtained from gazebo
            gzdbg << "AngleMax [deg] " << raySensor->AngleMax().Degree() << "\n";
            gzdbg << "AngleMin [deg] " << raySensor->AngleMin().Degree() << "\n";
            gzdbg << "RangeMax [m] " << raySensor->RangeMax() << "\n";
            gzdbg << "RangeMin [m] " << raySensor->RangeMin() << "\n";
            gzdbg << "AngleResolution [deg] " << raySensor->AngleResolution() * 180.0 / M_PI << "\n";
            gzdbg << "RangeCount " << raySensor->RangeCount() << "\n";
            gzdbg << "UpdateRate " << raySensor->UpdateRate() << "\n";

            //Almacena el puntero al modelo
            model = _model;
            if (!_sdf->HasElement("left_joint"))
                gzerr << "MyWheels plugin missing <left_joint> element\n";

            if (!_sdf->HasElement("right_joint"))
                gzerr << "MyWheels plugin missing <right_joint> element\n";

            leftJoint = _model->GetJoint(_sdf->GetElement("left_joint")->Get<std::string>());
            rightJoint = _model->GetJoint(_sdf->GetElement("right_joint")->Get<std::string>());

            if (!leftJoint)
                gzerr << "Unable to find left joint[" << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
            if (!rightJoint)
                gzerr << "Unable to find right joint[" << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";

            //Conecta la funcion OnUpdate al evento de actualizacion del mundo
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MyWheels::OnUpdate, this));
        }

        //Declara a inicializa la variable cont
        int cont=0;

        //Define la funcion OnUpdate que se llamará en cada iteracion de la simulacion
        void OnUpdate()
        {
            std::vector<double> ranges;
            raySensor->Ranges(ranges);
            if (ranges.size()==0) return;
            gzdbg << std::accumulate(ranges.begin(), ranges.end(), 0.0) / ranges.size() << "\n";

            //Obtiene la Posicion y Orientacion
            ignition::math::Pose3d pose = model->WorldPose();

            //Las imprime por pantalla
            printf("Posicion actual: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
            printf("Orientacion actual: %f %f %f\n", pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());

            printf("\nDeteccion Derecha: %f\n", ranges[20]);
            printf("\nDeteccion Izquierda: %f\n", ranges[700]);
            printf("\nDeteccion Frente: %f\n", ranges[360]);

            //Establece una velocidad constante para el modelo
            double speed = 5.0;

            //Logica de control
            if (cont==0)
            {
                //Si se encuentra una pared de frente o esta demasiado cerca de la pared, Gira a la izquierda
                if (ranges[360]<0.5 || ranges[20]<0.2){
                    printf("Gira izquierda\n");
                    leftJoint->SetVelocity(0, -speed);
                    rightJoint->SetVelocity(0, speed);
                }else{
                    //Si detecta una pared a la derecha, Sigue adelante
                    if (ranges[20]<0.5){
                        printf("Hacia delante\n");
                        leftJoint->SetVelocity(0, speed);
                        rightJoint->SetVelocity(0, speed);  
                    //Si no detecta nada a la derecha ni de frente, Gira hacia la derecha para volver a encontrar pared                      
                    }else{
                        printf("Gira derecha\n");
                        leftJoint->SetVelocity(0, speed);
                        rightJoint->SetVelocity(0, 1.0);                          
                    }
                }

            }   

            // Una vez llega a la meta, deja de moverse e imprime FIN DEL RECORRIDO
            if (pose.Pos().X()>=8 && pose.Pos().Y()>=10.5)
            {
                leftJoint->SetVelocity(0, 0);
                rightJoint->SetVelocity(0, 0);
                printf("\n\nFIN DEL RECORRIDO\n\n");
                cont=3;
            }        
        }

    private:
        physics::ModelPtr model; 
        physics::JointPtr leftJoint, rightJoint;
        sensors::RaySensorPtr raySensor;
        sensors::SensorPtr sensor;             // Pointer to the sensor
        event::ConnectionPtr updateConnection; 
    };

    //Registra el plugin al simulador
    GZ_REGISTER_MODEL_PLUGIN(MyWheels)
}
```

## [VIDEO-PLUGIN 2 map2.world](https://www.youtube.com/watch?v=9i43arTW6N4)

