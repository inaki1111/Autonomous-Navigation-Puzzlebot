# Proyecto a Futuro de la Materia

# Navegación Autónoma en ROS

Este proyecto implementa un sistema de navegación autónoma para un robot móvil utilizando el Sistema Operativo de Robots (ROS).

## Descripción

El objetivo futuro de este proyecto es permitir que un robot móvil navegue de manera autónoma en un entorno desconocido, evitando obstáculos y alcanzando destinos especificados. El sistema de navegación se basa en la integración de varios componentes de ROS, incluyendo la percepción del entorno, la planificación de trayectorias y el control del movimiento.

- **Detección de Paredes y Obstáculos:** Se utiliza un LiDAR para detectar las paredes y los obstáculos en el mapa, permitiendo al robot evitar colisiones durante la navegación.
- **Detección de Marcadores ArUco:** Se emplea una cámara para detectar marcadores ArUco en el entorno, lo que ayuda al robot a determinar su posición relativa y orientación.
- **Filtro de Kalman:** Se implementa un filtro de Kalman para fusionar la información del LiDAR y la cámara y mejorar la precisión de la estimación de la posición y orientación del robot.

## Requisitos del Sistema a Futuro

- ROS (Sistema Operativo de Robots) instalado. Se recomienda la distribución ROS Noetic.
- Un robot móvil compatible con ROS.
- Sensores de percepción del entorno, como cámaras o sensores láser.

## Actividad 1 de este proyecto a futuro.

- Descripcion : Realizar jerarquia de carpetas para que funcionen archivos .launch, marcadores en Rviz (navegacion y mapeo),directorios y modelos de mapa tanto de robot a futuro.

- Para esta primera actividad del proyecto, necesitas de realizar lo siguiente:

1. Clona este repositorio en tu espacio de trabajo de ROS:


2. Compila el espacio de trabajo de ROS: `catkin_make`


3. Asegúrate de que todos los paquetes necesarios estén instalados y configurados correctamente en tu entorno de ROS.


4. Toma en cuenta que cuando agregues nuevos archivos o carpetas asegurate de compilar el espacio de trabajo.

## Uso

1. Inicia el sistema de ROS:


2. Asegurate de tener la carpeta `puzzlebot_gazebo` en la ruta `home/user/catkin_ws/src` reemplazar user por tu nombre de usuario.


3. Copiar la carpeta `map_test` en la carpeta oculta dentro de home que podemos visualisar utilizando el comando `ctrl + H` con el nombre `.gazebo`. Una vez encontrado pegala dentro de la carpeta `models`.


3. ejecuta el comando desde la terminal `roslaunch arrow_autonomous_navigation_puzzlebot simulacion.launch`

## Contribuciones

Las contribuciones son bienvenidas. Si deseas contribuir a este proyecto, sigue estos pasos:

1. Haz un fork del repositorio.
2. Crea una nueva rama (`git checkout -b feature/nueva-caracteristica`).
3. Realiza tus cambios y haz commit de ellos (`git commit -am 'Agrega nueva característica'`).
4. Haz push de la rama (`git push origin feature/nueva-caracteristica`).
5. Abre una solicitud de extracción en GitHub.

## Autores

- Enrique Martinez Luna A01552626
- Iñaki Roman Martinez A01702712
- Salvador Yair Gallegos Lopez 01707962 A01707962

## Licencia

Este proyecto está bajo la Licencia MIT. Consulta el archivo [LICENSE](LICENSE) para obtener más detalles.
