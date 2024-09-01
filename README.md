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
  2. cd marus-example <br/>
- Pull git submodules <br/>
  1. git submodule update –init --recursive <br/>
