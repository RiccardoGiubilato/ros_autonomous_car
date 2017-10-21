## towards a ROS powered autonomous car using cheap arduino-like hardware

This project aims to build an autonomous vehicle exploiting the cheapest sensors and devices in the market.
I intend to keep this repository alive and constantly updated with every little changes and improvements I can make during my free time.

If you think this code can help to build your own ROS powered rc car please feel free to let me know. You are also encouraged to fork this repo and bring your own contribution to the project!!

##### Brief overview of the car:
* Cheap rc car chassis from ebay (20€)
* L298H motor driver (5€)
* Rotary encoder for the arduino (2€)
* Ultrasonic sensor (2€)
* Crius SE 2.6 board (30€ used)
* Power distribution board for drones with BEC (5€)
* Udoo quad running ROS (100€) -> Can be replaced by a Raspberry PI3 which is way cheaper!
* Nuts - bolts - spacers - connectors (<10 €)

##### Nodes
* _odom.py_: subscribes to the topic published by the arduino. A Vector3 message contains the wheel encoder values, the yaw from the IMU and the ultrasonic range. The nodes computes and publishes odometry
* _teleop.py_: listens to Joy messages and publishes velocity commands to the arduino
* _sonar-to-laser.py_: publishes range data as Range and LaserScan (just one scan) messages
* _laser-to-occupancy-grid.py_: map the environment based on odometry data and range data. It does not perform SLAM, just pure mapping

##### Launch
* _car-bringup.launch_: define tfs and launches odometry and teleoperation
* _car-mapping.launch_: run mapping node

##### Notes
If you find this code useful and you want to check my recent publications, please visit my 
ResearchGate page at this [link](https://www.researchgate.net/profile/Riccardo_Giubilato).
If you have any questions, my email address is riccardo.giubilato@gmail.com
