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

