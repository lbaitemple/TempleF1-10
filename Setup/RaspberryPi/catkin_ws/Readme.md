This is a code to control the car using raspberry pi and IMU (https://www.sparkfun.com/products/14001)

```
roslaunch  rpimotor f1.launch
```
Pontentially, we can run it reomotely using the following launch file


<launch>
    <machine name="rpi" address="10.109.xx.xx" user="pi" password="raspberry" env-loader="~/catkin_ws//config/env.sh" default="true"/>
    <!-- just testing a simple node-->
    
</launch>
