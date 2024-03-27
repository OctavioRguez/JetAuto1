# JetAuto1

JetAuto1 is a project for controlling two Hiwonder JetAuto Pro Vehicles. It will enable the vehicles to reach consensus at the same time they go to a certain position by controlling their movement. 

This project is achieved by using a control for the omnidirectional robots velocity and simulating a virtual leader for the vehicles to follow. The control is a closedloop and it uses the estimated position from a camera 
in a controlled environment.

## Installation

To install and run this project you will need Python and the following Python libraries installed:

- numpy
- opencv-python (cv2)
- pupil-apriltags
- pygame
 
To install these libraries, you can use pip:

```bash
pip install numpy opencv-python pupil-apriltags pygame
```

Additionally, the project uses ROS melodic and the following depedencies:
- Rospy
- geometry_msgs (Twist)

## Usage

To activate the control, navigate to the directory containing the project files and run the following command:
```bash
    python control.py
```

For getting the data (position feedback for the controller) from and external device, you can modify the python script [`main.py`](scripts/main.py) and then execute it by creating a ROS project:
```bash
    cd ~/catkin_ws/src
    git clone https://github.com/OctavioRguez/JetAuto1.git
    mv ./JetAuto1 ./jetauto1
    cd ..
    catkin_make
    cd ~/catkin_ws/src/jetauto1/scripts
    chmod +x launchControl.sh
    ./launchControl.sh
```

## Demo
https://github.com/OctavioRguez/JetAuto1/assets/115122414/ee6ad2b3-3633-488b-823d-67bb2f5f7690

## License

This project is licensed under the BSD 3-Clause License. See the [LICENSE](LICENSE) file for details.
