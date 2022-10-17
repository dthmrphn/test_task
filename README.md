## Weather control station


### Changing parameters

Each node has own controlls: source - path to file containing values, period - frequency of measurement
Example: 
```
//shows nodes
ros2 node list

// sets period to 1000ms
ros2 param set /temperature period 1000

// changes source file
ros2 param set /relhumidity source values.txt
```
