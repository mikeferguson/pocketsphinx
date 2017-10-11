# ROS package for pocketsphinx  
Original repository: https://github.com/mikeferguson/pocketsphinx  
  
Also used repo: https://github.com/gorinars/ros_voice_control  
  
You can know more about pocketpshinx here: https://cmusphinx.github.io/  
  
This package is an attempt to bring offline speech recognition to ROS. Pocketsphinx already offers many easy-to-use features in this domain, hence this package can be considered as an extension of pocketsphinx in the ROS world!  

## Dependencies  
1) pyaudio  
    ```
    sudo pip install pyaudio
    ```  
    If this does not work, follow instructions below:
    ```
    sudo apt-get install libasound-dev
    sudo apt-get install python-pyaudio
    ```
2) pocketsphinx: You will need to have pip preinstalled for this to work
    ```
    sudo pip install pocketsphinx
    ```
    There are many dependencies which need to be met before installation of pocketsphinx through pip works.
    Use Synaptics package manager to install the unmet dependencies which would be mentioned as error messages on the terminal window in case installation fails. Some of them include:  
    libpulse-dev  
    swig

## Getting Started
Clone this repository into the src folder of your catkin workspace using:  
```  
cd ~/catkin_ws/src
git clone https://github.com/Pankaj-Baranwal/pocketsphinx
```
To know more about catkin workspace and ROS, follow instructions at: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment  
After everything is setup, open a terminal from your catkin workspace and type the following command:  
``` 
catkin_make
```
Now, you are all setup. Go through the sections in wiki to find out more about how to use this package!
