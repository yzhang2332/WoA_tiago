# TIAGo - Full Setup for HRI Competition

This is a repository for The University of Melbourne's ("Wizards of Aus") entry into the [Office Assistant Robot Competition](https://hri2024c.web.app/hri2014rc2.html) at the [2024 HRI Conference](https://humanrobotinteraction.org/2024/) held in Boulder, Colorado. The robotic platform used in the competition is the [TIAGo mobile manipulator](https://pal-robotics.com/robots/tiago/) from PAL robotics. 

## Installation

### 1. Create a ROS workspace
```shell script
mkdir -p ~/tiago_ws/src
```

### 2. Clone the Git repository
```shell script
cd ~/tiago_ws/src/
git clone --recurse-submodules https://github.com/yzhang2332/woa_tiago.git
```

### 3. Installing the required packages from apt
```shell script
sudo apt-get install libportaudio2 python3-pip net-tools cmatrix
```

### 4. Install requirements for YOLOv8 (Ultralytics) forked repo
To install the required Python libraries:
```shell script
cd ~/tiago_ws/src/woa_tiago/ultralytics_ros/
python3 -m pip install -r requirements.txt
```
Then, install the ROS dependencies:
```shell script
cd ~/tiago_ws/
rosdep install -r -y -i --from-paths .
```

### 5. Installing the required Python libraries
```shell script
cd ~/tiago_ws/src/woa_tiago/
pip install -r requirements.txt
```
NOTE: The above might output an error - `ERROR: flask 3.0.2 has requirement click>=8.1.3, but you'll have click 7.0 which is incompatible.`
If so, simply execute the following in the terminal:
```shell script
pip install flask
```

### 6. Building the packages
```shell script
cd ~/tiago_ws/
catkin build detection_msgs tiago_follow_person tiago_gpt4 tiago_nav ultralytics_ros
```

