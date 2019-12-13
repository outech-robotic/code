# Flowing clean robot architecture

## What is it?

Just a proof of concept for the architecture described here.

Nothing serious / language specific, this is just an architure POC.

Source: https://download.outech.fr/archi_robot.html

[![arch](https://raw.githubusercontent.com/outech-robotic/hl-flowing-clean-arch/master/docs/img/archi_robot.png)](https://download.outech.fr/archi_robot.html)


## Static checks

This projects uses:
 * pylint
 * mypy
 * yapf (run `make yapf` or use a plugin for your IDE)
 * pipenv (for dependency management)

**Run `make jenkins` to check if your project pass the tests.**

## How to set up on local machine?

Install pipenv 
```shell
sudo pip3 install pipenv
```

Install the dependencies
```shell
cd hl-flowing-clean-arch
pipenv install 
```

Enter into the virtual environment.
```shell
pipenv shell
```

If you want to run the code:
```shell
python -m src.main
```


You are done! (see pipenv documentation for how to do other stuff.)


## Simulation

Simulating the robot behaviour must be taken into consideration seriously when building the project.

We can use the same design pattern as the robot's code. 

![simulation arch](https://raw.githubusercontent.com/outech-robotic/hl-flowing-clean-arch/master/docs/img/simulation.png)

### The simulation controllers

The *SimulationRunner* is a class that runs an infinite loop which calculates the position of everything on the field N times per second.

The *SimulationController* creates events that will be processed by the *SimulationRunner*

For instance, when receiving a **Move forward** order, the *SimulationController* would create 100 events "Move forward". The *SimulationRunner* would execute them sequentially, tick after tick, until there is no event left.

### The robot and the simulation are symmetric

The simulation would actually be inverted compared to the robot's code. What comes out of the robot is the input for the simulation. The input of the robot is actually the simulation output.

Thus, the handler of the simulation is actually the gateway of the robot. The gateway of the simulation actually "talks" with the handler of the robot.


## Testing

All the mocks at in /src/conftest.py
