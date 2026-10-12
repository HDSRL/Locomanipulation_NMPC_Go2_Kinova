# Kinova Arm Troubleshooting Experiments
## HDSRL - Vittorio Pastore


This repository includes some troubleshooting experiments for the Kinova 4DOF arm.

To use the programs take the following steps (**Tested on Ubuntu 20.04**):

1. Clone the repository or download the zip folder to a location on your system

2. *(Optional)* Read the Kinova SDK documentation (installed with the Kinova Gen2 SDK from Kinova's website) to learn about the functions and API calls.

3. Open the Experiments folder [Found at ```/Experiments```]

4. Open any of the subfolders of interest

5. Right click and click on "Open in terminal." Alternatively you can navigate to this directory through terminal.

6. Execute the following bash commands, in order:
  
```
$ mkdir build && cd build
```

```
$ cmake ..
```

```
$ make
```

8. This should create a new build folder, run cmake, and create a new executable in the build folder. Now you can simply run the following bash command:

```
$ ./<Name of Executable>
```

