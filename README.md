# 📷🚗📡 Exercise 1 – Sensor Data Acquisition in CARLA
The goal of this exercise is to implement collect real-time sensor data with our own CARLA client. This setup simulates the data collection scenario that will be followed in posterior exercises to train autonomous driving models.

We focus on collecting data from two sensors:

<div align="center">

<table>
  <tr>
    <td align="center"><strong>RGB Camera</strong></td>
    <td align="center"><strong>LIDAR</strong></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/318b869e-dd27-40f7-a803-89766996a967" width="300"/></td>
    <td><img src="https://github.com/user-attachments/assets/aed5d579-8a93-4308-9e9f-ddaa2fba93f3" width="300"/></td>
  </tr>
</table>

</div>

Along with the camera and LIDAR images, the script also collects key vehicle state and control information:

- **Throttle**: The acceleration value from the autopilot.
- **Steering**: The current steering angle.
- **Velocity**: The ego vehicle’s speed in m/s.
- **Indication**: The driving intention sent by the traffic manager (e.g., turn left/right).

This information is recorded at every simulation tick and saved in a CSV file. 
### 🗂️ Deliverable Files
<div align="center">
  
| File / Folder        | Description                                      |
|----------------------|--------------------------------------------------|
| `images_xx.png`, ...   | RGB camera images per frame                     |
| `lidar_xx.png`, ...  | LIDAR top-down projections per frame            |
| `data.csv`           | CSV file with vehicle information data |
| `connect2carla.py`   | Python script used to run the data collection   |

</div>

# 🚗⇔🤖 Exercise 2 – Integrating CARLA with ROS2
The goal of this exercise is integrating the previous sensor data aquisition into a ROS2 environment.  
The client we implemented in Exercise 1 is adapted to act as a **ROS2 node**, publishing the acquired sensor and vehicle data through ROS2 topics instead of saving them directly to disk.

So the RGB camera, LIDAR and vehicle information are now published, which allows other ROS2 nodes to acces it, being able to visualize it with the `ros_visualize_drive.py` file.
<div align="center">
  
| Topic Name         | Message Type        | Description                                |
|--------------------|---------------------|--------------------------------------------|
| `rgb_cam`          | `CompressedImage`   | RGB image from front camera                |
| `lidar`            | `PointCloud2`       | Top-down LIDAR scan                        |
| `ego_speed`        | `Vector3Stamped`    | Vehicle speed (x field)                    |
| `ego_acceleration` | `Vector3Stamped`    | Throttle/brake value (x field)             |
| `ego_steering`     | `Vector3Stamped`    | Steering value (z field)                   |
| `tm_action`        | `Vector3Stamped`    | Driving command (z field: 1=Left, 2=Right) |

</div>

Additionally it supports the following control modes:
<div align="center">
  
| Mode        | Description                                 | Key |
|-------------|---------------------------------------------|-----|
| Manual      | Controlled with keyboard                    | M   |
| Autopilot   | Driven by CARLA’s traffic manager           | N   |
<!-- | AI  | Uses our own AI agent control developed in exercise 4 | B   | -->

</div>

### 🗂️ Deliverable Files
<div align="center">
  
| File                    | Description                                               |
|-------------------------|-----------------------------------------------------------|
| `connect2carla_ros.py`  | Main CARLA ROS2 node that captures and publishes data     |
| `ros_visualize_drive.py`| ROS2 visualization node showing camera/LIDAR and status   |

</div>

# 🚗⇔🌍 Exercise 3 – Scenario Generation with Scenic
The goal of this exercise is to generate targeted scenarios using the Scenic language in CARLA to expose failure cases for the CIL++ autonomous driving model. These scenarios simulate challenging edge-case situations that make the model fail. The scenarios with the correct maneuver could then be used to finetune a driving model and help it learn how to deal with these scenarios making the model more rebust.

📄 Scenic Scenario Descriptions

<div align="center">

| File                    | Description                                               | Video  |
|-------------------------|-----------------------------------------------------------|--------|
| `carlaCola.scenic`  | Lead car is an uncommon vehicle which drives slower in fron of ego car ||
| `last_min_trash_avoid.scenic`| Lead car performs last-moment lane change to avoid trash ||
| `traffic_lights.scenic`| Ego vehicle approaches an intersection ||
| `curva_pedestrian.scenic`| Ego vehicle enters a curve while a pedestrian suddenly crosses ||
| `running_side.scenic`| Pedestrian runs in fornt of the ego vehicle ||

</div>




