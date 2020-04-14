# Flowing clean robot architecture

## What is it?

Just a proof of concept for the architecture described here.

### Diagram

[![arch](https://raw.githubusercontent.com/outech-robotic/hl-flowing-clean-arch/master/docs/img/archi_robot.png)](https://raw.githubusercontent.com/outech-robotic/hl-flowing-clean-arch/master/docs/img/archi_robot.png)


## How to set up the project on local machine?

### Prerequisites :
The project runs on **python 3.7**, make sure you have it installed.


###  Install [pipenv](https://github.com/pypa/pipenv):
```shell
sudo pip3 install pipenv
```

### Install the dependencies:
```shell
cd hl-flowing-clean-arch
pipenv install # Equivalent to `pip install -r requirements.txt`
```

On Raspberry Pi you might have to: 
```
sudo apt-get install libatlas-base-dev
```

### Enter into the virtual environment:
```shell
pipenv shell # Equivalent to `source ./bin/activate`
```

### Run it!
##### If you want to run the code in a simulation:
```shell
make run-simulation
```
##### If you want to run the code in real life:
```shell
export STUB_LIDAR=true # If you don't have a LIDAR connected.
export STUB_SOCKET_CAN=true # If you don't have ISO-TP server online.

make run
```
Go to `https://nicolasbon.net/replay/?replay=ws://localhost:8080` to see what the robot "sees".

Replace localhost with your IP if you are not running it locally.

##### If you want to run the tests:
```shell
make jenkins
```

##### If you want to re-generate the protobuf files:
```shell
make protoc
```


You are done! (see [pipenv documentation](https://github.com/pypa/pipenv/blob/master/README.md) for how to do other stuff.)

## Scripts run on CI

Every time we make a pull request, a bunch of scripts are run on the code to make sure the code is valid.

Here is a list of there scripts and why we are using them.


| Name   | Why do we use it? |
| ------ | ---- |
| [pylint](https://www.pylint.org/) | Linting the code, making it consistent and readable for everyone. |
| [yapf](https://github.com/google/yapf)   | Consistent auto-formatting of the code. Run `make yapf` to automatically reformat the code or [install the plugin for your IDE](https://plugins.jetbrains.com/plugin/10960-yapf).|
| [mypy](http://mypy-lang.org/)   | Static type checker for python, makes sure we pass the right types. |
| [pipenv](https://github.com/pypa/pipenv) | Package manager for python, like pip, but better.|
 
 
**Run `make jenkins` to check if your project pass the tests locally.**



## Simulation

Simulating the robot behaviour must be taken into consideration seriously when building the project.

We can use the same design pattern as the robot's code. 

![simulation arch](https://raw.githubusercontent.com/outech-robotic/hl-flowing-clean-arch/master/docs/img/simulation.png)

### Simulation components.

The *SimulationRunner* is a class that runs an infinite loop which calculates the state of everything N times per second.

The *SimulationHandler* receives messages from the robot and creates events that will be processed by the *SimulationRunner*. These events will change the state of the simulation. They are put into the `EventQueue`.

For instance, when receiving a `Move the robot forward by 100 units` order, the *SimulationController* would create **100 events** `Robot moved forward by 1 unit on tick N` in the EventQueue.

The *SimulationRunner* would execute them sequentially, tick after tick, until there is no event left, modifying the simulation state each time.

### The robot and the simulation talk to eachother through the bus

> The **input** of the simulator is the **output** of the robot. The **output** of the simulator is the **input** of the robot.

![simulation arch](https://raw.githubusercontent.com/outech-robotic/hl-flowing-clean-arch/master/docs/img/simulation_is_independant.png)

The simulation is built with the same pattern than the robot's code (handler/controller/gateway). 

**What comes out of the robot is the simulation input. The input of the robot is the simulation output.**

Thus, the **handler** of the simulation receives data from the **gateways** of the robot. The **gateway** of the simulation actually send data to the **handlers** of the robot.

When running the simulation, we just run the robot's strategy controller and the simulation runner in __parallel__ and we just let them communicate.

### The simulator is a spy

In order to display the state of the world at each tick **as seen by the robot**, the simulator actually "spies" on the robot's **internal state**.

For instance, we can inspect where the robot think it is (odometry output), by looking at the position stored in the localization controller.


In order to do that cleanly, we use **probes**. 
_You can picture it as if you would attach a probe to an electronic circuit to get the voltage on a particular point on a circuit (ie. capacitor...)._

Just use the `simulation_proble.attach()` function, which takes as a first argument **the name of the probe**, and as a second argument a **function ([closure](https://en.wikipedia.org/wiki/Closure_\(computer_programming\))) that returns the value of the inspected variable**.

#### Example:
```python
# Require a simulation_probe from the dependency injection container.

simulation_probe.attach("angle", lambda: float(my_angle))
simulation_probe.attach(
    "position", lambda: {
        'x': float(my_position.x),
        'y': float(my_position.y),
    })
```


### Replaying the simulation 

If you launch the simulator, you will see that a webpage in your browser opens ([example here](https://nicolasbon.net/replay/?replay=https://replay-api.outech.fr/replay/e6f5f9ed-a5c9-40f8-9b98-fca56a0f9a2a)). It actually replays the state of the simulation so we can see what is going on.


#### How does it work?

When the simulator has finished running virtual match, it will automatically upload the simulation result online. 

The simulation result is a big JSON string that contains the state of the game for each tick.

##### Example
```json
{
  "initial_configuration": {
    "sizes": {
      "ROBOT_A": [
        240,
        380
      ]
    }
  },
  "frames": [
    {
      "time": 0,
      "robots": {
        "ROBOT_A": {
          "angle": 0,
          "position": {
            "x": 200,
            "y": 1200
          }
        }
      }
    },
    {
      "time": 15,
      "robots": {
        "ROBOT_A": {
          "angle": 0,
          "position": {
            "x": 200,
            "y": 1200
          }
        }
      }
    },
    {...},
    {...},
    {...},
    {...},
  }
}
    
```

Time is in milliseconds.

The content of "ROBOT_A" is actually the result of all the probes at a certain instant. 
```json
{
  "angle": 0,
  "position": {
    "x": 200,
    "y": 1200
  }
}
```



#### Displaying this JSON

We have a small javascript code that fetches this JSON and read it frame by frame in the browser.

If you are adding new probes and want to see the result in the browser, modify this code.

See: https://github.com/bonnetn/replay

## Testing

Most of the code is unit tested to make sure the code is as reliable as possible.

All the mocks/stubs are stored in `/src/conftest.py`.

See [pytest](https://docs.pytest.org/en/latest/) documentation to learn how to write test easily or just check the existing unit tests. 

## Picture / diagrams

All diagrams in this README were created using https://www.draw.io/.

You can find the .drawio file in the `docs/` folder.
