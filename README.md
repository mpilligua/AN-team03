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
| `carlaCola.scenic` | Lead car is an uncommon vehicle which drives slower in front of ego car. <br><br> Ego car does not correctly respect distance and hits the car. | <img src="https://github.com/user-attachments/assets/99279ac1-604c-458a-b031-9fa4280b1e7d" width="300"/> |
| `last_min_trash_avoid.scenic` | Lead car performs last-moment lane change to avoid trash. <br><br> Ego car does not manage to evade the obstacle and crashes. | <img src="https://github.com/user-attachments/assets/2551a757-3bad-4fbb-9b74-60191cbfd9b9" width="300"/> |
| `traffic_lights.scenic` | Ego vehicle approaches an intersection with a traffic light. <br><br> Ego car does not recognize traffic lights correctly and it goes through despite light being red. | <img src="https://github.com/user-attachments/assets/8fded9ca-bff7-483a-a824-5fe7fc54d9d6" width="300"/> |
| `curva_pedestrian.scenic` | Ego vehicle most of the times enters a curve while a pedestrian suddenly crosses. <br><br> Ego car does not stop and hits the pedestrian. | <img src="https://github.com/user-attachments/assets/1ff24775-c923-46d0-8cff-db6b3a5b83c2" width="300"/> |
| `running_side.scenic` | Pedestrian runs in front of the ego vehicle. <br><br> Ego car stops a first time, but then it no longer sees the pedestrian, does not stop and hits it. | <img src="https://github.com/user-attachments/assets/904e1280-14e9-4681-9366-8b4a9bc22886" width="300"/> |


| `obstacle_rightside_road.scenic` | There is an obstacle in the rightside of the road. <br><br> Ego car does not stop, neither evade it and crashes into the object. | <img src="https://github.com/user-attachments/assets/30df522c-aa34-4327-8f38-675e427a3ac6" width="300"/> |

</div>


# 🤖🏎️ Exercise 4 – Training a CIL++ Model

In this exercise, we have trained the **CIL++v2** using the data we've been collecting. The goal is to use the autopilot to generate a dataset, format it, and train a model that can drive in both seen and unseen conditions.

---

## 📥 Data Collection

We first need to gather data that the model can learn from. We collected two types of data:

### 🚗 1. Random driving with traffic 
- We use the **autopilot** mode in **Town01** to record driving data under 4 different weather conditions: `ClearNoon`, `ClearSunset`, `HardRainNoon`, and `WetNoon`. We recorded around 7 hours of data with traffic (100 cars and 150 pedestrians).

### 🚗 2. Random driving without traffic 
- Similarly to before, we record 2 hours of alone driving through **Town01** in order to achieve key actions like stoping in a traffic light without any car stoping in front of them and following the road wihout any guidance from other cars.

## 🎯 Model Evaluation
We have added in the repo the files needed to visualize our model: 
- connect2carla_ros_2, which sets the carla world, ego, and runs the model.
- ros_visualize_drive, which receives the data and shows it on the screen.

### 🌍 Town02 (unseen environment)
In the following videos you can see the model running in town02, this evaluates **generalization**:
*The videos are speed up to x1.5

https://github.com/user-attachments/assets/420c364b-38fc-42ce-abfa-0e2cb0da6ed5  


https://github.com/user-attachments/assets/1728fe35-72c4-4a51-801c-6d53118967eb


### Traffic ligths and correct turn metric: 
To have a quantitative way of comparing the models we have generated a script that automatically spawns the car in front of 2 different traffic lights with different indications ("right", "left", "followlane", "straight"), each scenario is repeated three times with each of the possible indications (there are no 4 way intersections, therefore there are always 2 correct possible indications plus lanefollow).
In the following [link](https://drive.google.com/file/d/1VIQYhUK-C4D0aVd1W4DrkxAiOVejYrVg/view?usp=drive_link),  you can watch the video of the scenarios and a comparison between the models. 

The final metrics are: 

|Model        | # Correct traffic lights                                 | # Correct turns |
|-------------|---------------------------------------------|-----|
| 1 camera (85 FOV)      | 0                   | 7   |
| 2 cameras (85 FOV + closeup)      | 1                   | 8   |
| 1 camera + loss + aug      | 6                   | 7   |

From this table, we can see that adding the augmentation and the weighted loss really improves with correctly stopping at traffic lights. 


---
