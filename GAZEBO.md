Autor: Javier Gómez Eguizábal

Fecha: 03/03/24

## DIMENSIONES DEL MAPA

```bash
(J A V I E R = 6 * 2 = 12) -> max_x = 12 

(G O M E Z = 5 * 2 = 10) -> max_y = 10
```

## MAPAS CREADOS
### [map.world](map.world.xml)
<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/b5b23037-ea4d-4707-8bff-b934f2c821ae" width = 30%/>
</p>

<p align="center">
  <img src="https://github.com/Javierge15/Simuladores-de-Robots/assets/148269271/91c935d2-e23d-4c95-b3cf-cad2dfdfc6d4" width = 60%/>
</p>


## ALGORITMO
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
## VIDEOS
### [VIDEO-map.world](https://youtu.be/4jBIY4Zbtvw)
