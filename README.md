# TELLO CONTROLLER
## Dependencies
<div align="center">

| Package | Instalation command |
|--------------|--------------|
| **Tello Driver** | `git clone --recursive https://github.com/appie-17/tello_driver.git` |
| **Camero Info Manager Py** | `git clone https://github.com/ros-perception/camera_info_manager_py` |
| **TelloPy Library** | `pip3 install tellopy`
|**ORB Slam2**|`git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git`|

</div>

### Install dependencies with Rosdep
- `rosdep install --from-paths src --ignore-src -r -y`

### Modify package
- Change all instances of python2 to python3 in tello_driver, you can do this with VSC

## Launching this package
| |Command|Description|
|---------|---------|---------|
|Launch tello driver node | `roslaunch tello_controller tello_driver_node.launch` | This command makes the connection with the drone
|Launch tello_controller, rqt_plot, rqt_reconfigure, input keyboard node  | `roslaunch tello_controller tello_control.launch` |This will launch the PD control node along with the rqt reconfigure window, rqt plot windows plotting reference values against real values (adjust axes fot better visualization) and the keyboard command node. Make sure to adjust gains and references values before taking off the drone to avoid mistakes.
|Launch ORB Slam package|`roslaunch orb_slam2_ros orb_slam2_tello_mono.launch`| <ins>*orb_slam2_tello_mono.launch*</ins> file has to be copied in <ins>*orb_slam2_ros/ros/launch*</ins> folder
| rqt_image_viewer | `rqt` | To visualize orb map

When everithing above is launched, on the tello_controller window, make the drone take off, once it stabilizes, wait for ORB algorithm to diverge. Once that happens, press `o` to fix the current coordenates as zero.<br />
Then you can set the references with the sliders and the PD will move the drone, or set a mission.<br />
Check what command the drone is following on the terminal.


## Keyboard commands:
- `t` to takeoff
- `space bar` to land
- `p` to stop motors immediately
- `h` to hover the drone
- `o`to activate orb slam coordenates
- `1` to command mission 1
    - When on mission, the drone stops reading references from rqt_reconfigure window, but it keeps reading the gains (kp and kd) for the controller
- `2` to command mission 2
- Any other key will keep the drone in `PD mode`, set the reference with the slider

## Parameter reconfigure:
You can set the following parameters using the sliders on the rqt_reconfigure window:
- PD Control gains
- Reference values

## Published system data:
&nbsp;&nbsp; **Topic:** &nbsp; `/system_vars`

which contains:

|Type|Name|Description|
|---------|---------|---------|
|float64|x_d|Desired reference of **x**
|float64|x|Real **x** value
|float64|y_d|Desired reference of **y**
|float64|y|Real **y** value
|float64|z_d|Desired reference **altitude**
|float64|z|Real **altitude** value
|float64|yaw_d|Desired reference **yaw** angle
|float64|yaw|Real **yaw** angle


