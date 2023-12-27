# Fire Extinguishing Robot
The motivation for this project is to develop a robotic system to navigate flames and extinguish them autonomously.
 
The project utilized the commercially available Firebird VI robot from Nex Robotics - India, augmented with key components:

Sensing System: Three infrared flame sensors connected to an Arduino UNO microcontroller detect the presence and approximate location of flames.

Pose Estimation: A Hokuyo Filter laser scanner, communicating with the Arduino via a rosserial connection, calculates the precise pose of the identified fire source.

Navigation: Employing the ROS navigation stack, the robot autonomously navigates towards the fire while avoiding any obstacles in its path.

Extinguishing Mechanism: Upon reaching the designated location, the robot activates its onboard water pump, effectively extinguishing the fire.

This project yielded valuable insights into the potential of robots for fire suppression in controlled environments. It demonstrated the feasibility of combining commercially available platforms with custom-developed control systems for targeted fire response. 


