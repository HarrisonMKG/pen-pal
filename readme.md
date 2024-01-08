<h1>PEN-PAL<sup>®</sup> Using Kinova Kortex </h1>

Steps to setup environment:
+Clone Repository 
```sh
  git clone https://github.com/HarrisonMKG/pen-pal.git
```
+Switch to Dev/Personal Branch 
```sh
  git checkout BRANCH_NAME
```
+IF virtual environment NOT setup (./venv/ folder) 
```sh
  python -m venv venv 
```

You will need to do the following each time you start a new terminal
+Activate Virtual Environment 
```sh
  source venv/Scripts/activate 
```
+Install neccessary packages from Requirements.txt 
```sh
  pip install -r requirements.txt 
```

To stop using virtual environment python"
```sh
  deactivate 
```


Look at Kinova_Readme.md and the Readme.md file in api_cpp/examples folder for instructions how to install cMake, gcc