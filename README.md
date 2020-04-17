# OUTech code

## Prerequisites 

Install protoc on your machine:
```shell
apt install protoc # OR pacman -S protobuf
```

### High level
The python projects run on **python 3.7**, make sure you have it installed.

####  Install [pipenv](https://github.com/pypa/pipenv):
```shell
sudo pip3 install pipenv
```
See [pipenv documentation](https://github.com/pypa/pipenv/blob/master/README.md) for how to work with it.

#### Install the python dependencies:
```shell
pipenv install # Equivalent to `pip install -r requirements.txt`
```

#### On Raspberry Pi you might have to: 
```
sudo apt-get install libatlas-base-dev
```

## Regenerate protobuf files:
```shell
make protoc
```

## Running the code
### High level
#### Enter into the virtual environment:
```shell
pipenv shell # Equivalent to `source ./bin/activate`
```

#### Run it!
##### If you want to run the highlevel code in a simulation:
```shell
make run-simulation
```
##### If you want to run the highlevel code in real life:
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

## Building
### Low Level

Install cmake, arm-none-eabi-{gcc, gdb, newlib, binutils}, or equivalents on your platform (this assumes archlinux or Windows)

On Linux, follow the installation procedure for openocd in their Readme (Compiling OpenOCD):\
https://sourceforge.net/p/openocd/code/ci/master/tree/

In the root directory, run the following to setup the make files and commands:
```shell script
cmake -B lowlevel/cmake-build-debug lowlevel
```

If using CLion, add the following CMake Options (```File```>```Settings```>```Build, Execution, Deployment```>```CMake```):\
```-DCMAKE_C_FLAGS=--specs=nosys.specs -DCMAKE_CXX_FLAGS=--specs=nosys.specs```\
This is because CLion does pre-checks on the compilers, and defaults to using the standard library, but doesn't link the one for ARM.

From the command line, still in the root directory, you can now make the following targets:

```make -C lowlevel/cmake-build-debug build_motor/build_motor_g4```\
Builds the Motor Board project, for Nucleo-F042K6 or the Nucleo-F042K6

```make -C lowlevel/cmake-build-debug flash_motor/flash_motor_g4```\
Uploads the corresponding program to the board, using ```openocd```.

From CLion, you can just use Build configurations, and use the build hammer on the ones needed.

## Debugging

We will be using ```arm-none-eabi-gdb``` as a client, and ```openocd``` as a server.
In CLion:
* In the previous Build settings, set the Debugger to ```arm-none-eabi-gdb```.
* Go to your ```Configurations``` (next to the build hammer/run button).
* ```Edit Configurations```
* Click ```+``` to add a new configuration > ```Embedded GDB Server```
* Give it the name you want.
* Select the build target executable you want.
* ```'target remote' args``` : ```localhost:PORT```
  * Choose the ```PORT```, preferably high (The default ```3333``` doesn't work on windows).
  * It is also possible to debug on a remote host.
* ```GDB Server``` : ```openocd```
* ```GDB Server args``` : We need to give it a path to a configuration file and also an alternative port (if needed, on windows)
  * ```-f "path_to_a_file_in_lowlevel/scripts/board or target" -c "gdb_port PORT"```
* Go to ```Advanced GDB Server options```.
  * Set the ```Working directory``` to the path of the Code project.
  
If the project is already buit, press the ```Debug``` button next to configurations, while the new configuration is selected.

Automation of this setup ASAP.