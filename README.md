Create a data type that looks like the following and name my_custom_message
```
│   my_custom_message
│       ├── bool bool_value
│       ├── int8 integer_value
│       │── float float_value
│       ├── String string_value
```
## Adding a custom message as a new package to the build system
First, we are going to create a new package, I assume that you have a ROS 2 Humble installation in your computer (let me know if not):

> ```shell
> # Go to this library folder installation
> cd ~/Arduino/libraries/micro_ros_arduino-0.0.1
> 
> # Go to the extra_packages folder and create there you new msg package:
> pushd extras/library_generation/extra_packages
> 
> # Here follow the instructions of this tutorial: https://micro-ros.github.io/docs/tutorials/core/create_new_type/
> ros2 pkg create --build-type ament_cmake my_custom_message
> [...]
> ```
 
At the end of this procedure you should have custom message inside the folder:
> ```shell
> extras/library_generation/
> ├── ...
> ├── extra_packages
> │   ├── extra_packages.repos
> │   └── my_custom_message
> │       ├── CMakeLists.txt
> │       ├── include
> │       │   └── my_custom_message
> │       ├── msg
> │       │   └── MyCustomMessage.msg
> │       ├── package.xml
> │       └── src
> └── ...
> ```
Regarding this new `extra_packages` folder we have two possibilities:
* Adding directly folders that contain ROS 2 packages
* Adding an entry to [this file](https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/extras/library_generation/extra_packages/extra_packages.repos), you can see a sample of this kind of files [here](https://github.com/micro-ROS/micro_ros_setup/blob/foxy/config/agent_uros_packages.repos). This option allows you to automatically download a Git repo every time the library is built.
 
If the packages that you are going to add are standard enough, maybe we can add them to this repo [extra_packages.repos](https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/extras/library_generation/extra_packages/extra_packages.repos).  
## Rebuilding your own version of the micro-ROS library for Arduino
Now we are going to use docker to build our custom version of the library. In this case, this custom version of the library will have the extra msg that you just have included, but for example, is also possible to modify the configuration of the micro-ROS layers by customizing some of the [.meta files](https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/extras/library_generation/colcon.meta).
> ```shell
> # Go to this library folder installation
> cd ~/Arduino/libraries/micro_ros_arduino-0.0.1
> 
> # Use the docker to build all the necessary stuff:
> docker pull microros/micro_ros_static_library_builder:humble
> docker run -it -v $(pwd):/arduino_project --net=host microros/micro_ros_static_library_builder:humble
> ```
> If you want to build just for your platform, use:
> 
> ```shell
> docker run -it --rm -v $(pwd):/arduino_project microros/micro_ros_arduino_builder:humble
> ```
> 
Then upload the 'Arduino code'

Start the Micro ROS agent 
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
Then echo the topic "/micro_ros_arduino_node_publisher" to see whether the topic is being published or not.
```
ros2 topic echo /micro_ros_arduino_node_publisher 
```
![image](https://github.com/krishna4104/RigBetel_Labs_Micro_Ros/assets/140909916/0de66791-aa81-4839-8f9a-a415a909f042)
