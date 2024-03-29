# OUTech code

## General 

Install protoc on your machine:
```shell
apt install protoc # OR pacman -S protobuf
```

Clone submodules:
```shell
git submodule init && git submodule update
```

Regenerate protobuf files:
```shell
make protoc
```

## High level
The python projects run on **python 3.7**, make sure you have it installed.

###  Install [pipenv](https://github.com/pypa/pipenv):
```shell
sudo pip3 install pipenv
```
See [pipenv documentation](https://github.com/pypa/pipenv/blob/master/README.md) for how to work with it.

### Install the python dependencies:
```shell
pipenv install -d # Equivalent to `pip install -r requirements.txt`
```

### On Raspberry Pi you might have to: 
```
sudo apt-get install libatlas-base-dev
```

### Running the code
#### Enter into the virtual environment:
```shell
pipenv shell # Equivalent to `source ./bin/activate`
```

#### Run it!
##### If you want to run the highlevel code in a simulation:
```shell
make run-simulation
```
##### If you want to run the highlevel code on hardware:
```shell
export STUB_LIDAR=true # If you don't have a LIDAR connected.
export STUB_SOCKET_CAN=true # If you don't have ISO-TP server online.

make run
```
Go to `https://outech-robotic.github.io/replay/?replay=ws://localhost:8080` to see what the robot "sees".

Replace localhost with your IP if you are not running it locally.

##### If you want to run the tests:
```shell
make jenkins
```

You are done! 

## Low Level
### Prerequisites
Install cmake, arm-none-eabi-{gcc, gdb, newlib, binutils}, or equivalents on your platform (this assumes archlinux or Windows)

On Linux, follow the installation procedure for openocd in their Readme (Compiling OpenOCD):\
https://sourceforge.net/p/openocd/code/ci/master/tree/

### Build & Upload
#### Setup CLion
Open the root ```code``` directory as the root of the project.\
In the project hierarchy, right-click on ```lowlevel/CMakeLists.txt``` and "Load Cmake project".

For Windows only:\
Add the following CMake Options (```File```>```Settings```>```Build, Execution, Deployment```>```CMake```):\
```-DCMAKE_SYSTEM_NAME=Generic -DCMAKE_C_FLAGS=--specs=nosys.specs -DCMAKE_CXX_FLAGS=--specs=nosys.specs```\
This is because CLion uses a default system name for CMake that makes it use the full standard library, but we don't want any of that.

#### Run CMake from CLI
In the root directory, run the following to setup the make files and commands:
```shell script
cmake -B lowlevel/cmake-build-debug lowlevel
```

From CLion, you can just use Build configurations, and use the build hammer on the ones needed.

#### Build the projects on CLI
From the command line, still in the root directory, you can now make the following targets:

```make -C lowlevel/cmake-build-debug build_motor/motor_g4/servo/servo_nucleo```\
Builds the required project.

```make -C lowlevel/cmake-build-debug flash_motor/motor_g4/servo/servo_nuclo```\
Uploads the corresponding program to the board, using ```openocd```.

The  flash_ targets should automatically detect modifications and rebuild if needed, as they have their respective build_ target as a dependency.

### Debugging

We will be using ```arm-none-eabi-gdb``` as a client, and ```openocd``` as a server.
In CLion:
* Go to your ```Configurations``` (next to the build hammer/run button).
* ```Edit Configurations```
* Click ```+``` to add a new configuration > ```Embedded GDB Server```
* Give it the name you want, preferably different than the existing targets.
* Select the build target & executable you want.
* 'target remote' args: ```localhost:PORT``` (PORT being the port that will be used for GDB)
* GDB Server: ```openocd```
* GDB Server args: ```-f lowlevel/scripts/board/BOARD.cfg -c "gdb_port PORT"``` (BOARD being one of the board files used)
* In Advanced GDB Server options:
  * Working directory: Set it to the root directory of the project (code)

If the project is already built, press the ```Debug``` button next to configurations, while the new configuration is selected.

Automation of this setup is a WIP.
