import sys
import numpy as np

import carla
import time
import numpy as np
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from sensor_msgs.msg import CompressedImage, PointCloud2
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import String

from cv_bridge import CvBridge

import sys
sys.path.append('/home/team03/AN-team03/CILv2-demo/code/CILv2_multiview')
from cilv2_wrapper import CILv2Wrapper


HOST = "localhost"
PORT = 2350
MAP_NAME = "Town02" # Test also in 'Town02' to check generalization
WEATHER =  "SoftRainSunset" # "ClearNoon" # Test also in 'SoftRainSunset' and 'WetSunset' to check generalitzation
EGO_VEHICLE= "vehicle.dodge.charger_2020"
TRAFFIC_MANAGER_PORT = PORT + 6050

GEN_DATA = False

PATH_WEIGHTS = f"/home/team03/AN-team03/Exercise4/Results_V6/_results/configs/CILv2_Fov85NewLoss/checkpoints/CILv2_multiview_attention_40.pth"
PATH_CONFIG = f"/home/team03/AN-team03/Exercise4/Results_V6/_results/configs/CILv2_Fov85NewLoss/CILv2_Fov85NewLoss.yaml"

do_manual = False
do_async = False

cv_bridge = CvBridge()

# Class to define the simulation mode
class Mode(Enum):
    MANUAL = 0
    AUTO = 1
    AI = 2

def carla_image_to_compressedimage(carla_image, node, time_stamp):
    array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
    array = array.reshape((carla_image.height, carla_image.width, 4))
    array = array[:, :, :3]
    ci_msg = CompressedImage()
    ci_msg = cv_bridge.cv2_to_compressed_imgmsg(array, dst_format='jpg')
    ci_msg.header.stamp = time_stamp
    return ci_msg

def carla_lidar_to_pointcloud2(carla_lidar, node, time_stamp):
    pc_msg = PointCloud2()
    pc_msg.header.stamp = time_stamp
    points = np.frombuffer(carla_lidar.raw_data, dtype=np.dtype('f4'))
    pc_msg.height = 1
    pc_msg.width = points.shape[0]*points.itemsize
    pc_msg.point_step = 4*points.itemsize
    pc_msg.row_step = points.shape[0]*points.itemsize
    pc_msg.is_dense = True
    pc_msg.data = carla_lidar.raw_data
    return pc_msg


class CarlaNode(Node):
    def __init__(self):
        super().__init__('CARLA_node_03')

        # Inizialize the client and register the publishers
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.rgb_publisher = self.create_publisher(CompressedImage, "rgb_cam", qos_profile= qos_policy)
        self.lidar_publisher = self.create_publisher(PointCloud2, "lidar", qos_profile= qos_policy)
        self.speed_publisher = self.create_publisher(Vector3Stamped, "ego_speed", qos_profile= qos_policy)
        self.action_publisher = self.create_publisher(Vector3Stamped, "tm_action", qos_profile= qos_policy)

        self.steering_publisher = self.create_publisher(Vector3Stamped, "ego_steering", qos_profile= qos_policy)
        self.acceleration_publisher = self.create_publisher(Vector3Stamped, "ego_acceleration", qos_profile= qos_policy)

        if not GEN_DATA:
            # Subscribers for the manual control (Actions are given at ros_visualize_drive.py)
            self.accel_subscriber = self.create_subscription(Vector3Stamped, "acceleration", self.callback_accel, qos_profile=qos_policy)
            self.steer_subscriber = self.create_subscription(Vector3Stamped, "steering", self.callback_steer, qos_profile=qos_policy)

            # Subscriber for the simulation mode (manual or automatic)
            # The mode is set in the ros_visualize_drive.py file, where the user can choose between manual and automatic driving
            self.mode_subscriber = self.create_subscription(Vector3Stamped, "mode", self.callback_mode, qos_profile=qos_policy)
            # Subscriber for the AI model actions
            self.action_subscriber = self.create_subscription(String, "action", self.callback_action, qos_profile=qos_policy)
            # Loading the defined model
            self.model = CILv2Wrapper(PATH_CONFIG, PATH_WEIGHTS)

        
        # Define a timer for the carla handler
        SIM_FPS = 20
        SIM_PERIOD = 1/SIM_FPS
        self.timer = self.create_timer(SIM_PERIOD, self.carla_handler)
        
        # Variable initialization
        self.carla_image = None
        self.lidar_points = None
        self.ros2_accel = 0.0
        self.ros2_steer = 0.0
        self.action = 0.0
        self.i = 0
        self.WEATHER = WEATHER

        # Initizlize carla
        self.init_carla()

        # Set the start mode
        self.mode = Mode.AUTO if not do_manual else Mode.MANUAL
        self.do_manual = do_manual
        
        # Save the satrt time of the simulation
        self.START_TIME = time.time() 

    def init_carla(self):
        print("Configuring the client")
        self.client = carla.Client(HOST, PORT)
        self.client.set_timeout(30.0)
        self.client.load_world(MAP_NAME)

        print("Setting world parameters")
        self.sim_world = self.client.get_world()
        self.sim_world.set_weather(eval(f'carla.WeatherParameters.{self.WEATHER}')) 

        print("STARTING WORLD WITH WEATHER: ", self.WEATHER)
        settings = self.sim_world.get_settings()
        settings.synchronous_mode = not do_async # Enables synchronous mode
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 16
        self.sim_world.apply_settings(settings)

        print("Creating the Ego vehicle")
        blueprint_library = self.sim_world.get_blueprint_library()
        ego_bp = blueprint_library.find(EGO_VEHICLE)

        self.traffic_manager = self.client.get_trafficmanager(TRAFFIC_MANAGER_PORT)
        self.traffic_manager.set_synchronous_mode(True)

        spawn_points = self.sim_world.get_map().get_spawn_points()
        spawn_point = spawn_points[np.random.randint(0, len(spawn_points))]

        self.ego = self.sim_world.try_spawn_actor(ego_bp, spawn_point)

        # Camera sensor
        cam_bp = self.sim_world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x",str(300))
        cam_bp.set_attribute("image_size_y",str(300))
        cam_bp.set_attribute("fov",str(85))
        cam_location = carla.Location(x=1.2, y=0, z=1.9)
        cam_rotation = carla.Rotation(pitch=-20, yaw=0, roll=0)
        cam_transform = carla.Transform(cam_location,cam_rotation)

        self.ego_cam = self.sim_world.spawn_actor(cam_bp,cam_transform,attach_to=self.ego, attachment_type=carla.AttachmentType.Rigid)
        self.ego_cam.listen(lambda image: self.rgb_callback(image))

        # Lidar sensor 
        lidar_bp = self.sim_world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(1))
        lidar_bp.set_attribute('lower_fov', str(-5))
        lidar_bp.set_attribute('upper_fov', str(1))
        lidar_bp.set_attribute('rotation_frequency', str(20))
        lidar_bp.set_attribute('range', str(100))

        lidar_location = carla.Location(x=2, y=0, z=0.75)
        lidar_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
        lidar_transform = carla.Transform(lidar_location,lidar_rotation)

        self.ego_lidar = self.sim_world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.ego, attachment_type=carla.AttachmentType.Rigid)
        self.ego_lidar.listen(lambda points: self.lidar_callback(points))

        self.ego.set_autopilot(not do_manual, TRAFFIC_MANAGER_PORT)

        self.dict_actions = {'LaneFollow':4.0, 'Left': 1.0, 'Right':2.0, 'Straight':3.0} 
        print('executing updated version of connect2carla.py')

    # save the recived data using the callbacks
    def rgb_callback(self, image): self.carla_image = image
    def lidar_callback(self, lidar_points): self.lidar_points = lidar_points
    def callback_accel(self, data): self.ros2_accel = data.vector.x
    def callback_steer(self, data): self.ros2_steer = data.vector.z
    def callback_mode(self, data): self.mode = Mode(data.vector.x)
    def callback_action(self, data): self.action = data.data

    def carla_handler(self):
        # Get a timestamp to publish the data
        time_stamp = self.get_clock().now().to_msg() 
        
        # if the simulation is running in async mode, wait for the tick
        # else, tick the simulation
        if do_async:
            self.sim_world.wait_for_tick()
        else:
            self.sim_world.tick()

        # Get and publish the camera image
        if not(self.carla_image is None):
            msg = carla_image_to_compressedimage(self.carla_image, node, time_stamp)
            self.rgb_publisher.publish(msg)

        # Get and publish the lidar points
        if not(self.lidar_points is None):
            msg = carla_lidar_to_pointcloud2(self.lidar_points, node, time_stamp)
            self.lidar_publisher.publish(msg)

        # Get and publish speed
        v_speed = self.ego.get_velocity()
        speed = math.sqrt(v_speed.x**2 + v_speed.y**2 + v_speed.z**2)
        msg_speed = Vector3Stamped()
        msg_speed.header.stamp = time_stamp
        msg_speed.vector = Vector3()
        msg_speed.vector.x = float(speed)
        self.speed_publisher.publish(msg_speed)

        # Get control
        control = self.ego.get_control()

        # Update autopilot mode if needed
        if (self.mode == Mode.MANUAL) != self.do_manual:
            self.do_manual = self.mode == Mode.MANUAL
            self.ego.set_autopilot(not self.do_manual, TRAFFIC_MANAGER_PORT)
        
        # If manual mode, set the acceleration and steering from the ROS2 topic
        if self.mode == Mode.MANUAL:
            if (self.ros2_accel >= 0):
            
                control.throttle = self.ros2_accel
                control.brake = 0
            else:
                control.throttle = 0
                control.brake = -self.ros2_accel
            
            control.steer = self.ros2_steer
            self.ego.apply_control(control)
            
        # If automatic mode, get the action from the traffic manager
        elif self.mode == Mode.AUTO: 
            # Get and convert action
            try:
                action = self.traffic_manager.get_next_action(self.ego)[0]
                if action in self.dict_actions.keys():
                    action = self.dict_actions[action]
                else:
                    action = 0.0
            
            except IndexError:
                action = 4.0
                print("No action available")

            # Publish action
            msg_action = Vector3Stamped()
            msg_action.header.stamp = time_stamp
            msg_action.vector = Vector3()
            msg_action.vector.z = float(action)
            self.action_publisher.publish(msg_action)

        # If AI mode, get the action from the model
        elif (self.mode == Mode.AI) and not GEN_DATA:
            data = {}
            image = np.frombuffer(self.carla_image.raw_data, dtype=np.uint8)
            image = image.reshape((self.carla_image.height, self.carla_image.width, 4))

            # Changing the image format from BGRA to RGB
            image = image[:, :, :3][:, :, ::-1]
            data['img1'] = image
            data['speed'] = float(speed)

            # print(self.action)
            # print(self.dict_actions.keys())

            # Getting the action from given through ros2 subscriber
            if self.action in self.dict_actions.keys():
                data['direction'] = self.dict_actions[self.action]
            else:
                raise ValueError(f"Action {self.action} not available. Choose between {self.dict_actions.keys()}.")
            
            # Processing the data for the model and getting the prediction
            data = self.model.preprocess_data(data)
            pred = self.model.forward(data)

            # Appling the predicted control to the ego vehicle
            if (pred['acceleration'] >= 0):
                control.throttle = float(pred['acceleration'])
                control.brake = 0
            else:
                control.throttle = 0
                control.brake = -float(pred['acceleration'])
            
            control.steer = float(pred['steer'])
            self.ego.apply_control(control)
        else:
            if (self.mode == Mode.AI) and GEN_DATA:
                raise ValueError("AI mode is not available in data generation mode.")
            else:
                raise ValueError(f"Mode {self.mode} is not available. Choose between MANUAL, AUTO or AI.")

        # Publish acceleration
        msg_acceleration = Vector3Stamped()
        msg_acceleration.header.stamp = time_stamp
        msg_acceleration.vector = Vector3()
        msg_acceleration.vector.x = float(control.throttle)
        self.acceleration_publisher.publish(msg_acceleration)

        # Publish steering
        msg_steering = Vector3Stamped()
        msg_steering.header.stamp = time_stamp
        msg_steering.vector = Vector3()
        msg_steering.vector.z = float(control.steer)
        self.steering_publisher.publish(msg_steering)


if __name__ == "__main__":
    rclpy.init()
    node = CarlaNode()
    rclpy.spin(node)
