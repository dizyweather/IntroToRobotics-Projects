Hello random viewer!

This Repo is to show what I did in my intro to robotics class; programming and simulating robots in Webots! I'll give you a rundown of what I learned/did and a short video to see the simulation working.

EPuck
- The Epuck Robot is a simple robot, a "puck" with 2 wheels
- I read data from it's distance sensor to control behavior
- Attached IR sensors to the front to detect and follow a line

Tiago
- The Tiago robot is a much more complex robot with LIDAR, optional arms, and cameras.
- First I learned how to gather, transform, and store LIDAR point clouds.
- Using those point clouds to generate a probablistic map
- Create a configuration space (cspace) using the probablistic map
- Able to use pathfinding algorithms like A* to generate paths on the cspace
- Learned about rapidly exploring random search
- Implemented behavior trees to control the robot in a grab and drop objects task

You can download the files here and open the .wbt file in the worlds folder to open it in webots and test it for yourself!
(Assuming you have all the libraries installed: numpy, py_trees)
