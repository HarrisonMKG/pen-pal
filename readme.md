<h1>PEN-PAL<sup>®</sup> Using Kinova Kortex </h1>
<h2>Description</h2>

<h2>Build Environment</h2>

Steps to setup environment:

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

You will need to do the following each time you start a new terminal
+ Activate Virtual Environment 
```sh
  source venv/Scripts/activate 
```
+ Install neccessary packages from Requirements.txt 
```sh
  pip install -r requirements.txt 
```
+ Configure Conan:
```sh
conan config set general.revisions_enabled=1

conan profile new default --detect
```

+ To stop using virtual environment python"
```sh
deactivate 
```


TO BUILD:
+ Change directory to the Examples folder (Or the top level that contains the kortex_api and utilities.cpp file)
```sh
cd api_cpp/examples
```
+ Execute build script:
```sh
./scripts/build-mingw.bat release
```

+ Run specific examples/files:
```sh
BUILD_FOLDER/EXECUTABLE_FILE.exe
```

When there are changes to the files, go into the BUILD_FOLDER and run the MakeFile
```sh
cd BUILD_FOLDER
make
```
(If make is not recognized, use 'mingw32-make')

Look at Kinova_Readme.md and the Readme.md file in api_cpp/examples folder for instructions how to install cMake, gcc

<h2>Usage</h2>

Modify "coordinates.csv" in terms of:

```
time_1,x_1,y1,z_1
time_2,x_2,y_2,z_2
...
time_n,x_n,y_n,z_n
```

Executuion:
Navigate to the `src` directory and run the following command:
```
./build-gcc-release/bin/my_file
```
Further arguements can also be passed:
```
-h,--help : provides a list of arguments
-o,--output : provide output directory for log files
-c,--coordiantes : provide csv coordiante data
-g,--gain : provide file name for the gain values to be used for each PID controller
```

./bin/300-Controller_Wrapper_100-Pen-Pal-Demonstration.exe  -g ../gain_values/gain_2.txt -c ../coordinates/line_X_up_down_short.csv -r Y

./bin/300-Controller_Wrapper_100-Pen-Pal-Demonstration.exe  -g ../gain_values/gain_2.txt -c ../coordinates/line_zig_zag.csv


## Laser Tracking
### Description
This is a script for demo purposes only. The script will take a video of a laser pointer with no other light source on a flat source and output the following:
- Time Tagged X,Y waypoints
- IK Computation of waypoints
- Graph of Waypoints
### Usage
`python laser_tracking.py <.mp4 of laser pointer video> -i <Number of points to be linearly interpolated (i<=1)>`
### Considerations
- Have no other light sources during recording (everything should be black but the red dot)
- Use a red laser pointer
