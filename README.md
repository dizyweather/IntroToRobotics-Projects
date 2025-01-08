# Intro To Robotics - Projects

Hello random viewer!

This Repo is to show what I did in my intro to robotics class; programming and simulating robots in Webots! I'll give you a rundown of what I learned/did and a short video to see the simulation working.
The majority of my code are located in the controllers folders

EPuck
- The Epuck Robot is a simple robot, a "puck" with 2 wheels
- I read data from it's distance sensor to control behavior
- Attached IR sensors to the front to detect and follow a line

Tiago
- The Tiago robot is a much more complex robot with LIDAR, optional arms, and cameras.
- First I learned how to use homogenous transformations to transform our LIDAR point clouds in world coordinates.
- Using those point clouds to generate a probablistic map
- Create a configuration space (cspace) using the probablistic map
- Able to use pathfinding algorithms like A* to generate paths on the cspace
- Learned about rapidly exploring random search
- Implemented behavior trees to control the robot in a grab and drop objects task

You can download the files here and open the .wbt file in the worlds folder in webots and test it for yourself!
(Assuming you have all the libraries installed: numpy, py_trees)

I have attached some videos of the simulations running, hope you enjoy.

-----
### EPuck Waypoint Following
In addition to making the epuck follow a line, we also made the epuck follow set waypoints

https://github.com/user-attachments/assets/dd11bd0b-82cc-4a68-813b-e94de47252d1


### Tiago - Mapping and Localization
Here you can see the tiago following set waypoints and using a lidar to map out the enviornment, and at the end displaying a configuration space.

https://github.com/user-attachments/assets/b7a59b9a-7d78-44da-989a-cf05b73f9df3


### Tiago - A* Navagation
The start is the same as the previous but we now use that configuration space to A* to any set paths.
This version also uses a behavior tree to do mapping, navagation, and checking if we already mapped out the enviornment, and if so, just skip to doing A* (as seen in the ending).

https://github.com/user-attachments/assets/1565161c-a11b-4e30-adf3-fed8f1367560



### Tiago - Arm Manipulation
Explained in the video :D

https://github.com/user-attachments/assets/cef009b3-ae6f-48b4-a824-9549656b9b84








## License

[MIT](https://choosealicense.com/licenses/mit/)


