# sawyer-rubiks

### Workstation Setup
```
Follow this link. Make sure to use Ubuntu 20.04: 
https://support.rethinkrobotics.com/support/solutions/articles/80000980134-workstation-setup

Make sure the robot is running in SawyerSDK mode. If it isn't, then the program will not work. You can tell based on what's on the robot's screen after it wakes up. If there's a face, then it's not in SDK mode.
```

### Installation
```
sudo apt install python3.11
pip install opencv-python
python3 -m pip install -U pip
python3 -m pip install -U matplotlib
pip install kociemba
```

### Usage
```
Place cube with middle white square facing away from the robot and green middle square facing down.
The camera must be facing with its bottom towards the robot.

Run `roscore` in a terminal to start roscore

In a new terminal, run
`python3 src/main.py`
To start the solve. 
```

### Known Problems
```
- Sometimes the error of cube placement accumulates and certain moves will cause the cube to sit out of place in the mount.
- The control box will time out after extensive use. Reboot the control box to fix. 
```
