Autonomous Navigation in ROS
This project implements an autonomous navigation system for a mobile robot using the Robot Operating System (ROS).

Description
The future goal of this project is to enable a mobile robot to autonomously navigate an unknown environment, avoid obstacles, and reach specified destinations. The navigation system is based on the integration of various ROS components, including environment perception, trajectory planning, and motion control.

Wall and Obstacle Detection: A LiDAR is used to detect walls and obstacles on the map, allowing the robot to avoid collisions during navigation.

ArUco Marker Detection: A camera is employed to detect ArUco markers in the environment, helping the robot determine its relative position and orientation.

Kalman Filter: A Kalman filter is implemented to merge the information from the LiDAR and the camera, improving the accuracy of the robot's position and orientation estimation.

SLAM Navigation Implementation: This repository also includes the implementation of SLAM (Simultaneous Localization and Mapping) navigation for the robot, enabling it to create maps of its surroundings and localize itself within those maps.

Unknown Terrain Navigation Algorithms: The repository also contains algorithms for navigating in an unknown environment, such as the Bug 0 algorithm, which helps the robot find a path to a goal even when the terrain is unknown.

Contributions
Contributions are welcome. If you'd like to contribute to this project, follow these steps:

Fork the repository.

Create a new branch (git checkout -b feature/new-feature).

Make your changes and commit them (git commit -am 'Add new feature').

Push the branch (git push origin feature/new-feature).

Open a pull request on GitHub.

Authors
Enrique Martinez Luna A01552626

Iñaki Roman Martinez A01702712

Salvador Yair Gallegos Lopez A01707962

License
This project is licensed under the MIT License. See the LICENSE file for more details.
