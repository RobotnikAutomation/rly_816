# rly_816
ROS package to control I/O of the usb device RLY816

# Dependencies

* [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs.git)

# Available params

* port: port used for the connection
* outputs: number of available outputs (normally between 2 and 8)

# Launch the node

```
$ roslaunc rly_816 rly_816.launch
```

# Topics

## Status

To read the inputs/outputs status

```
/rly_816_node/status (type robotnik_msgs/inputs_outputs)

```

# Services

## Set the outputs value

```
/rly_816_node/set_digital_outputs (type robotnik_msgs/set_digital_output)

```

