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


Look at Kinova_Readme.md and the Readme.md file in api_cpp/examples folder for instructions how to install cMake, gcc

<h2>Usage</h2>

Modify "coordinates.csv" in terms of:

```
x_1,x_2,x_3,...,x_n
y_1,y_2,y_3,...,y_n
z_1,z_2,z_3,...,z_n
```