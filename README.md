# wind
 
This package contains the source code for reading incoming wind data from a 3D sonic anemometer. The anemometer is an Applied Technologies [SATI Type A probe](http://www.apptech.com/wp-content/uploads/2016/04/SATItechdesc_g.pdf).
 
Currently, the node sends a trigger character to the Sonic Anemometer which in turn sends a package of data to the Raspberry Pi. The anemometer to Pi communication is via Termios Serial Port libraries. As of 2/3/2020 the fastest the wind node can sample wind speeds is 100 hz. It can Sample up to 200 hz but some packets are dropped due to serial port timeing issues that are yet to be worked out. 

To run only the sonic anemometer and stream data run the following command:
```
roslaunch wind sonic_only.launch
````
