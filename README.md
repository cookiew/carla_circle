# carla_circle


run simulation for CARLA simulation with leq game planning and baseline w/o game



- In the CARLA folder  ```./CarlaUE4.sh -carla-server -windowed -ResX=640 -ResY=480 -quality-level=Low```

- launch two vehicles from carla_circle workspace
```python3 manual_control.py --positionx 14.1 --positiony 14.1 --yaw -45 --rolename stanford_hero --color "255,100,100"```   
```python3 manual_control.py  --positiony -4.5 --yaw 180 --rolename stanford_ego --color "255,200,200" --positionx 40.6```

- launch planner and pass in parameters
```roslaunch carla_circle leq_game_planner.launch p1rs:=10.0 p2rs:=10.0```

- ```roslaunch carla_circle leq_game_bridge.launch```

- ```roslaunch carla_circle leq_game.launch```



### carla version / carla ros version
0.9.5

##### Notes
- this controller directly publishes carla\_vehicle\_control message
- call julia functions from Python rosnode is very slow, do not know why

- call julia function inside a callback function does not work

- no timer is available so the planner is called through ros rate

- currently runs at 3Hz without problems 
