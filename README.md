# neroFly

This package provides a Webots-based simulator that replicates the dynamics of the Parrot Bebop 2 drone. 



## API Parity

This simulator operates independently and **does not require the actual hardware driver to function**. However, it is designed with **API parity** to the ROS 2 `bebop_autonomy` driver. 

If you are looking for the original hardware drivers, you can find the related repositories here:

- [ros2_bebop_driver](https://github.com/BraJavSa/ros2_bebop_driver)
- [ros2_parrot_arsdk](https://github.com/BraJavSa/ros2_parrot_arsdk)

## Launch Files

The package includes the following launch files:

| Launch File | Description |
|---|---|
| `bebop_launch.py` | Launches the Webots simulator with the Bebop drone and its visualizer. |
| `pio_bebop_launch.py` | Launches a scenario with both the Bebop drone and a Pioneer 3-AT rover. |


## ROS 2 Topics

### Publishers

| Topic | Message Type | Description |
|---|---|---|
| `/bebop/position` | `geometry_msgs/msg/PointStamped` | X, Y, Z position of the drone in the world. |
| `/bebop/altitude` | `std_msgs/msg/Float64` | Current altitude of the drone. |
| `/bebop/imu` | `sensor_msgs/msg/Imu` | IMU data (orientation, angular velocity, linear acceleration). |
| `/bebop/odom` | `nav_msgs/msg/Odometry` | Odometry data of the drone. |
| `/bebop/camera/image_raw` | `sensor_msgs/msg/Image` | Raw images from the drone's forward-facing camera. |
| `/bebop/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration information. |

### Subscribers

| Topic | Message Type | Description |
|---|---|---|
| `/bebop/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands for the drone. |
| `/bebop/takeoff` | `std_msgs/msg/Empty` | Command the drone to take off. |
| `/bebop/land` | `std_msgs/msg/Empty` | Command the drone to land. |
| `/bebop/emergency` | `std_msgs/msg/Empty` | Command the drone to instantly cut the motors. |
| `/bebop/ref_vec` | `std_msgs/msg/Float64MultiArray` | Data to position the reference visualizer (TargetVisualizer). |
| `/pioneer/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands for the Pioneer 3-AT (if launched). |
