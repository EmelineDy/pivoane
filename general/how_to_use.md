# This file describes the main steps to use the geicar

# Contents
1. **Quick start guide**

2. **Detailed manual**
*  1. Connection to the car and start of the ROS Nodes
*  2. How to drive the car in manual mode (with the XBOX Controller)
*  3. ROS2 tips

# 1. Quick start guide
This part describes how to start the car, how to drive in Manual mode with the joystick, and how to switch to the Autonomous Mode

1. Power ON the car 

2. Connect to the raspberry pi (you need to be on the same network, i.e the IoT network by default):\
**Adapt the IP address according to the car, password : geicar**
```sh
ssh pi@10.105.1.XX
```
*If you have problems connecting to the Raspberry, please refer to the detailed manual, section 2.i.b)*

3. Start raspberry ROS2 nodes :
```sh
ros2 launch geicar_start geicar.launch.py
```
Information about the nodes will be displayed, as well as a report describing the status of the system. \
As the ROS environment on the Jetson has not yet been started, **it is normal for the report to display errors concerning the Jetson (including the camera and the lidar)** (see the file [debugging_help](./debugging_help.md) for more details about the system report)

**The car is ready to be used in manual mode**

4. _**Optional, but recommanded on first use** You can start the steering calibration by pressing "DPAD Bottom" + "START". Then follow the instructions given in the launch terminal (ros2 launch) on the Raspberry._ (see [nucleoF103_software](../nucleoF103/documentation/software/software_description.md) for more details about the calibration)

5.  Drive the car with the XBOX controller :
    * Press "start" to start the car
    * Press "LT" to move backward, "RT" to move forward, and control the steering with the left joystick
    * Press "B" to stop the car
    * Press "A" to select Autonomous Mode (that do nothing by default), press "Y" to select Manual Mode

6. **Open existing docker container**
Due to the presence of many packages and management of versions we are using a docker container to run ROS. However we cannot open a graphic interface directly in a container, hence we should do a connection to the container from ssh connection.

Start docker container and enable ssh connection
```
docker start -ai ros-car
/etc/init.d/ssh restart
```
Open a new terminal and connect to container using ssh connection (in this new terminal it is possible to open graphic interface to see the camera image)

```
ssh -Y root@<docker_ip> -p <number_of_configured_port>
```

### Run launch file inside docker container
Due to the presence of two cameras we developed two launch file, one for each camera. Execute one of this launch inside the docker container connected by ssh to see the camera image.
```
ros2 launch darknet_ros yolov7-tiny-myClasses-video0.launch.py
```
or
```
ros2 launch darknet_ros yolov7-tiny-myClasses-video2.launch.py
```

7. **Optional** If you want to start the LIDAR and the CAMERA (not used by default):
      * Open a new terminal, and connect to the raspberry :\
      **Adapt the IP address according to the car, password : geicar**
      ```sh
      ssh pi@10.105.1.XX
      ```
      * Connect to the Jetson Nano board :\
      **password : geicar**
      ```sh
      ssh jetson@192.168.1.10
      ```
      * Go into the docker container "ros-humble" :\
      **password : geicar**
      ```sh
      sudo docker start -ai ros-humble
      ```
      * Start LIDAR and CAMERA nodes:
      ```sh
      ros2 launch geicar_start_jetson geicar.jetson.launch.py
      ```
**The LIDAR should be running, and the CAMERA should be ON**\
You can check that everything is working by pressing the "DPAD Left" button on the joystick. This displays a system report on the           launch terminal (on the raspberry) (see the file [debugging_help](./debugging_help.md) for more details about the system report)





# 2. Detailed manual

## i. Connection to the car and start of the ROS Nodes

### a)Powering the car
1. Plug in the battery
2. Press the ON/OFF red button
3. Press the "start" red button 

**All the system is now powered**


### b)Connection to the Raspberry PI
By default, the raspberry pi board connects to the IoT network at startup. 

_Sometimes the raspberry can't connect to the IoT network. This may be a problem with the network itself, or it may be that the date/time in the raspberry has drifted too much (there is no RTC). It is therefore advisable to add your own access point (e.g. your phone) to always be able to access the board.\
It is also possible to connect the board to any other network by modifying the file "/etc/netplan/50-cloud-init.yaml". You can edit this file directly from the SD card, or from a terminal on the raspberry :_
```sh
sudo nano /etc/netplan/50-cloud-init.yaml
sudo netplan apply
sudo reboot
```


1. Connect your pc to the same network as the raspberry (by default the IoT network)
2. Connect to the raspberryPi via ssh **(adapt the IP address according to the car and the network used)** : 
```sh
ssh pi@10.105.1.XX
```

3. Enter the password : "geicar"

**You are now in the raspberry environment : the prompt is now "pi@geicar"**


### c)Starting the ROS nodes (in the Raspberry PI board)
Start all necessary nodes :
```sh
ros2 launch geicar_start geicar.launch.py
```

Information about the nodes will be displayed, as well as a report describing the status of the system. \
As the ROS environment on the Jetson has not yet been started, **it is normal for the report to display errors concerning the Jetson (including the camera and the Lidar)** (see the file [debugging_help](./debugging_help.md) for more details about the system report)\

**The nodes on the raspberry are now started. You can control the car with the XBOX controller**


### d)Connection to the Jetson Nano
You can access the Jetson Nano board from the Raspberry. You have to establish a first ssh connection between the pc and the raspberry (step n°2), then establish the ssh connection between the raspberry and the jetson.

In another terminal :
1. Repeat step n°2 to connect to the raspberry again (you can check that you are in a Raspberry Pi terminal with the prompt, which indicates "pi@geicar")
2. From the raspberryPi terminal (with prompt "pi@geicar"), you can connect to the jetson nano board :
```sh
ssh jetson@192.168.1.10
```
3. Enter the password : "geicar" 

**The prompt is now "jetson@geicar"**


### e)Starting the ROS nodes (in the Jetson Nano board)
Once you are connected to the Jetson Nano (prompt jetson@geicar) :

1. Start and go into the docker container "ros-humble" (password : "geicar") : 
```sh
sudo docker start -ai ros-humble
```

**The prompt is now "root@geicar"**

2. From the docker container (prompt "root@geicar"), you can start the nodes that are in the jetson board (ie LIDAR and CAMERA) :
```sh
ros2 launch geicar_start_jetson geicar.jetson.launch.py
```

**You can now see that the LIDAR is running and the camera is on**


### f)Stop the nodes and turn OFF the car
1. Stop the nodes by pressing "Ctrl-C" in the launch terminals (in the Raspberry and in the Jetson)
2. Exit ssh connections :
```sh
exit
```
3. Turn OFF the car by pressing the ON/OFF red button on the car


## ii. How to drive the car in manual mode (with the XBOX Controller)
Once you have started the nodes in the Raspberry PI (step n°3), you can drive the car with the joystick in Manual Mode **(make sure the car can move safely)** :

1. Power ON the joystick
2. _**Optional, but recommanded on first use** You can start the steering calibration by pressing "DPAD Bottom" + "START". Then follow the instructions given in the launch terminal (ros2 launch) on the Raspberry._ 
3. Press "start" (you can see that the car is started in the launch terminal (ros2 launch) on the Raspberry
4. You can select the mode : button "Y" for Manual (by default at startup), button "A" for Autonomous
5. Control the car with LT, RT, and the left joystick
6. Press "B" to stop the car


## iii. ROS2 tips

### See all messages exchanged by ROS nodes (including sensors data)
Once you have started the nodes (step n°3 and/or step n°5), you can see the different topics and messages published

In a Raspberry Pi terminal or in a Jetson Nano terminal :
1. You can see the list of all the topics (sometimes you need to run this command twice) :
```sh
ros2 topic list
```
2. You can display the messages published in the topic :
```sh
ros2 topic echo 'NAME_OF_THE_TOPIC'
```
For example :
```sh
ros2 topic echo /us_data
```

### Visualize the lidar and the camera in rviz2
It is difficult to launch rviz2 through an ssh connection (too slow). However, you can **connect to the Jetson via HDMI to launch rviz2**.

1. **rviz2 is installed on the jetson (not in the docker container!)**. To launch it, simply run :
```sh
rviz2
```

2. Then start the ROS nodes of the jetson (in the docker):
```sh
sudo docker start -ai ros-humble
ros2 launch geicar_start_jetson geicar.jetson.launch.py
```

3. In rviz2, to vizualize the lidar scan:
      - Click on "Add" -> "by topic" -> "/scan"
      - In "global options", put "laser" in the field "Fixed Frame".

4. In rviz2, to visualize the camera image:
      - In " global options ", put "base_link" in the field "Fixed Frame", and "10" in "Frame Rate" (or corresponding frame rate)
      - Click on "add" -> "by topic", and select the topic "/image_raw" (image)
