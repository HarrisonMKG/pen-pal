# PEN-PAL<sup>®</sup> Using Kinova Kortex

## Description

This README should serve as a guide to use this repository, it will cover:

+ Cloning and setting up environment for the first time
+ Building project
+ Running examples
+ Making changes
+ Extra Parts/Notes of the System

## Cloning and Setting up Environment
#### First time setup steps:

+ Clone Repository 

```sh
  git clone https://github.com/HarrisonMKG/pen-pal.git 
```

+ Switch to Dev/Personal Branch 
```sh
  git checkout BRANCH_NAME
```

+ IF virtual environment NOT setup (./venv/ folder) 
```sh
  python -m venv venv 
```
+ Activate Virtual Environment 
```sh
  source venv/Scripts/activate 
```
+ Install neccessary packages from Requirements.txt 
```sh
  pip install -r requirements.txt 
```
+ Configure Conan (required for the kinove API):
```sh
conan config set general.revisions_enabled=1

conan profile new default --detect
```

+ To stop using virtual environment python
```sh
deactivate 
```

## Building Project
#### You will need to do the following each time you you want to compile changes in project.
 **When any changes happen to any of the examples, classes, or header file**
 
TO BUILD:
+ Change directory to the src folder
```sh
cd src
```
+ Execute build script:
  + If release is not added to the end, the name of the build folder will change and could do unexpected things during the compilation
  + The build scripts used was developed by Kinova and uses a naming convention to only include specific files for the CMAKE build
    + The script only grabs cpp files located within the project folder, inside of a directory that follows this REGEX expression **[0-9]*-*/**, and the filename follows the REGEX expression **[0-9]*.cpp**
  
```sh
./scripts/build-mingw.bat release
```

When there are changes to the files, go into the BUILD_FOLDER and run the MakeFile
**Note: when there is a new file to be included in the build you must re-run the original build script. The 'make' command below will not include the new files**
```sh
cd BUILD_FOLDER
make
```
(If make is not recognized, use 'mingw32-make')



## Running Examples
To run any of the current examples for this system, at the very least a coordinate and a gain file must be provided. The coordinate file contains the different waypoint trajectories the contain the positions the Kinova Gen 3 arm must reach during execution. The gain file will populate each actuator with the correct PID gain values to control them more accurately.

A waypoint coordinate file, for example  "coordinates.csv" is in the form of:
```
time_1,x_1,y1,z_1
time_2,x_2,y_2,z_2
...
time_n,x_n,y_n,z_n
```

A gain file, for example "gain.txt" is in the form of:
```
P_gain_1 I_gain_1 D_gain_1
P_gain_2 I_gain_2 D_gain_2
...
P_gain_6 I_gain_6 D_gain_6
```

Executuion:
Navigate to the `src` directory and run the following command:
```
./BUILD_FOLDER/bin/EXAMPLE.exe
```
Further arguements can also be passed:
```
-h,--help : provides a list of arguments
-o,--output : provide output directory for log files
-c,--coordiantes : provide csv coordiante data
-g,--gain : provide file name for the gain values to be used for each PID controller
-r,--repeat : (Y/N) Indicate if the system should repeat the trajectory once done **Note: this option should only be used for trajectories that end at the same point they start at, or else the transition wont be smooth**
```

### Usage

```
./bin/300-Controller_Wrapper_100-Pen-Pal-Demonstration.exe  -g ../gain_values/gain_2.txt -c ../coordinates/line_X_up_down_short.csv -r Y
```

```
./bin/300-Controller_Wrapper_100-Pen-Pal-Demonstration.exe  -g ../gain_values/gain_2.txt -c ../coordinates/line_zig_zag.csv
```

### Existing Examples
#### 100-Pen-Pal-Demonstration.cpp

+ This example executes the trajectory provided from the cartesian waypoint coordinate file. 
+ It first loads the coordinates, calculates the IK, executes the trajectory (should wait until space is pressed start), Generates the performance results (RMS vlaues as well as the ouptput graphs)
#### 101-save-trajectories.cpp

+ This example is to save a list of target joint angles from a provided trajectory to a CSV file
+ Using the IK function in the system, this example feeds the Cartesian waypoints to the system, applies a bias vector based on the starting position, then calculates the joint angles the arm would need for each point.
+ The naming convention for the output file uses the root name of the provided cartesian waypoint files and then adds "__joints" and then the starting X, Y, and Z position of the robot arm to know where it should be executed from
  + Example: if waypoint file is `coordinates.csv` and starting position is `Xpos, Ypos, and Zpos` the resulting file would be `coordinates__joints_Xpos_Ypos_Zpos.csv`
+ The output CSV file can be used with example **102-move_joint_angles.cpp** as the input coordinate file to execute the trajectory without needing to recalculate the IK
#### 102-move_joint_angles.cpp

+ Similar to example **100-Pen-Pal-Demonstration.cpp** but instead uses target joint angle CSV files as coordinates and executes them from there
+ for this example generate the performance results and compare it to the original trajectory you must ensure 2 thing:
  + The original cartesian waypoint file (used to generate the joint angles) must be in the same directory and have the same root name for both files (**coordinates**.csv and **coordinates**__joints_Xpos_Ypos_Zpos.csv)
 
#### 103_execute_demo.cpp

+ This example was purely used for the open house demo where multiple trajectories had their IK angles pre-calculated from different starting positions, and this example executes all of them in order
+ To move between the different trajectories, high-level control is used on the robot to move to Preset poses that can be set using the Kinova Admin page when connected to the robot (Look to adding a new action for the robot)

## Extra Parts/Notes of the System
### Gain values

+ Using the gain values in the .txt files and feeding them in each example was done so that you do not need to run the make command each time you change a value
+ It also allows to provide different profiles for the system (some moving faster than others, more accurate, using torque vs velocity...)
+ This approach should be considered to apply for the different configurations of the system (position tolerance, actuator control type, max & min velocity/torque) to prevent needing to constantly run the make command each time especially when tuning. 

### Logger class
Within the src/300-Controller-Wrapper directory there is a Logger class that was meant to be used as a class to log all messages coming from the system and save them for possible future reference. The base of the class is setup and was tested earlier in the semester but we did not fully integrate it due to some compatibility issues when running the system on Linux. 
+ This class could be useful as it can classifying messages into different categories, limit which messages get outputted to the terminal, and save the important messages to specific files

### Laser Tracking
#### Description
This is a script for demo purposes only. The script will take a video of a laser pointer with no other light source on a flat source and output the following:
- Time Tagged X,Y waypoints
- IK Computation of waypoints
- Graph of Waypoints
#### Usage
`python laser_tracking.py <.mp4 of laser pointer video> -i <Number of points to be linearly interpolated (i<=1)>`
#### Considerations
- Have no other light sources during recording (everything should be black but the red dot)
- Use a red laser pointer
