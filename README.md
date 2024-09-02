# Unity-Segmentation
In this project we will use PCL library and ROS2 to segment objects in Unity scene.
# Preparation
## Installing Unity <br/>
Install Unity Hub - https://unity.com/download <br/>
In Unity Archive, find 2021.3.3f1 and install: <br/>
  - Click on the Unity Hub button next to 2021.3.3 version <br/>
  - Installation window in Unity Hub should open <br/>
  - Follow instructions to install the editor <br/>
## Cloning Marus Example
ReadMe also available here: https://github.com/MARUSimulator/marus-example <br/>
- If you don't have one, create a GitHUb account <br/>
- Before cloning, make sure to setup ssh with your GitHub account and git-lfs installed <br/>
  1. https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account <br/>
  2. https://git-lfs.com <br/>
- Clone marus-example <br/>
  1. git clone git@github.com:MARUSimulator/marus-example.git <br/>
- Position yourself in the previously cloned repository <br/>
  1. cd marus-example <br/>
- Pull git submodules <br/>
  1. git submodule update –init --recursive <br/>
## Importing marus-example into Unity

 - Open Unity Hub <br/>
 - Click Add and select the root folder of marus-example <br/>
 - Select Unity version 2021.3.3f1 and open the project <br/>
 - Wait until first load is done <br/>

 ## Installing ROS2
 Install ROS 2 humble <br/>
 - https://docs.ros.org/en/humble/Installation.html <br/>

 ## Cloning grpc-ros-adapter and running the server 
 ReadMe also available here: https://github.com/MARUSimulator/grpc_ros_adapter/tree/galactic <br/>

1. Create colcon workspace in WSL <br/>

2. In workspace src folder clone grpc-ros-adapter a. git clone git@github.com:MARUSimulator/grpc_ros_adapter.git <br/>

3. Position yourself in the previously cloned repository <br/>
   - cd grpc_ros_adapter <br/>

   - switch branch: git switch galactic <br/>

4. Pull git submodules <br/>

a. git submodule update –init --recursive <br/>
 
5. install requirements <br/>

   - optionally create python virtual environment for your workspace and then: <br/>

   - pip install -r requirements.txt <br/>

6. In workspace src folder clone uuv_sensor_msgs
   - git clone git@github.com:labust/uuv_sensor_msgs.git <br/>

   - cd uuv_sensor_msgs <br/>

   - switch branch: git switch galactic <br/>

7. build with colcon build in your workspace root <br/>

8. source install/setup.bash <br/>

9. Start grpc server: ros2 launch grpc_ros_adapter ros2_server_launch.py <br/>
