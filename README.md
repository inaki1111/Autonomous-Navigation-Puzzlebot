# Navegación Autónoma en ROS

Este proyecto implementa un sistema de navegación autónoma para un robot móvil utilizando el Sistema Operativo de Robots (ROS).

## Descripción

El objetivo de este proyecto es permitir que un robot móvil navegue de manera autónoma en un entorno desconocido, evitando obstáculos y alcanzando destinos especificados. El sistema de navegación se basa en la integración de varios componentes de ROS, incluyendo la percepción del entorno, la planificación de trayectorias y el control del movimiento.

## Requisitos del Sistema

- ROS (Sistema Operativo de Robots) instalado. Se recomienda la distribución ROS Melodic o superior.
- Un robot móvil compatible con ROS.
- Sensores de percepción del entorno, como cámaras o sensores láser.
- Paquetes de navegación de ROS, como `move_base`, `map_server` y `amcl`.

## Instalación

1. Clona este repositorio en tu espacio de trabajo de ROS:


2. Compila el espacio de trabajo de ROS:


3. Asegúrate de que todos los paquetes necesarios estén instalados y configurados correctamente en tu entorno de ROS.

## Uso

1. Inicia el sistema de ROS:


2. ejecuta el comando desde la terminal (`roslaunch arrow_autonomous_navigation_puzzlebot simulacion.launch`)

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
