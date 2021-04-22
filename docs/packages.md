#### ros2 packages

---

###### publisher
*contains the turtlebot3 library and the planner*

```
src/publisher/src/publisher/publisher
├── __init__.py
├── libplanner
│   └── planner.py
├── libturtlebot3
│   ├── common.py
│   ├── logger.py
│   ├── movement.py
│   ├── sensor.py
│   └── turtlebot3.py
└── main.py
```

* `libplanner`
	* `planner.py`: *planner logic: guides the simulated virtual instance of the turtlebot3
		robot through the simulated virtual environment implementing obstacle detection and
		collision avoidance*
* `libturtlebot3`:
	* `common.py`: *global common variables definition module*
	* `movement.py`: *movements definition module*
	* `turtlebot3.py`: *container class, entry point module*
	* `logger.py`: *logger module*
	* `sensor.py`: *laserscan sensor reader module*
* `main.py`: *program entry point*

---

###### subscriber
*builds the entire simulated virtual environment*

```
src/subscriber/src/subscriber
├── launch
│   └── empty_world.launch.py
├── models
│   └── maze_1
│       ├── model.config
│       └── model.sdf
└── worlds
    └── empty_waffle_pi.model
```

* `launch`:
	* `empty_world.py`: *launch file (started by gzserver), calls the correct .world file and
	  creates the virtual instance of the turtlebot3 robot*
* `models`:
	* `maze_1`: *maze_1 model (.config and .sdf files) used within the simulation*
* `worlds`:
	* `empty_waffle_pi.model`: *world file*

---

###### simulator
*builds simulator client w/ 3d models definition*
