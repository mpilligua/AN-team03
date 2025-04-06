# We have added the possibility of changing driving mode: 
# key M for MANUAL, key N for AUTO, key B for AI

import argparse
import sys
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2
from geometry_msgs.msg import Vector3Stamped, Vector3

import numpy as np
from cv_bridge import CvBridge
import os

import pygame
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_LEFT
from pygame.locals import K_RIGHT
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_d
from pygame.locals import K_s
from pygame.locals import K_w
from pygame.locals import K_m
from pygame.locals import K_n
from pygame.locals import K_b

from enum import Enum


WIDTH, HEIGHT = 800, 600
TIMER_PERIOD_SEC = 0.05

ZOOM_IN = 15

class Mode(Enum):
    MANUAL = 0 # Key: M
    AUTO = 1 # Key: N
    AI = 2 # Key: B

class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


## function to process the arguments
def parse_args():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-t', '--tm_port',
        metavar='T',
        default=8000,
        type=int,
        help='TCP TrafficManager port to listen to (default: TCP port + 6000)')
    
    args = argparser.parse_args()

    return args


class ManualControlNode(Node):

    def __init__(self):
        super().__init__("manual_control_node_03")

        self.screen = None
        self.clock = None
        self._font_mono = None
        self._notifications = None
        self.mode = Mode.AUTO
        
        self.throttle = 0.0
        self.brake = 0.0
        self.acceleration = 0.0
        self.steering = 0.0
        self._steer_cache = 0.0
        self.cv_bridge = CvBridge()

        self.ego_speed = 0.0
        self.ego_accel = 0.0
        self.ego_steer = 0.0
        self.tm_action ="none"

        self.rgb_frame = None
        self.lidar_frame = None
        
        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.get_logger().info("Creating RGB image subscriber")
        self.rgb_subscriber = self.create_subscription(CompressedImage, "rgb_cam", self.callback_rgb_image, qos_profile=self.qos_policy)

        self.get_logger().info("Creating LiDAR subscriber")
        self.lidar_subscriber = self.create_subscription(PointCloud2, "lidar", self.callback_lidar_data, qos_profile=self.qos_policy)

        self.get_logger().info("Creating ego status subscribers")
        self.speed_subscriber = self.create_subscription(Vector3Stamped, "ego_speed", self.callback_ego_speed, qos_profile=self.qos_policy)
        self.accel_subscriber = self.create_subscription(Vector3Stamped, "ego_acceleration", self.callback_ego_acceleration, qos_profile=self.qos_policy)
        self.steer_subscriber = self.create_subscription(Vector3Stamped, "ego_steering",self.callback_ego_steering, qos_profile=self.qos_policy)
        self.action_subscriber = self.create_subscription(Vector3Stamped, "tm_action",self.callback_tm_action, qos_profile=self.qos_policy)    
        

        self.get_logger().info("Creating acceleration/steering publishers")
        self.pub_accel = self.create_publisher(Vector3Stamped, "acceleration", qos_profile=self.qos_policy)
        self.pub_steer = self.create_publisher(Vector3Stamped, "steering", qos_profile=self.qos_policy)
        self.pub_mode = self.create_publisher(Vector3Stamped, "mode", qos_profile=self.qos_policy)


        self.get_logger().info("Initialising window system")
        self.init_ui()


        self.get_logger().info("Creating timer")
        self.create_timer(TIMER_PERIOD_SEC, self.timer_callback)

    def callback_ego_speed(self, msg_speed: Vector3Stamped):
        self.ego_speed = msg_speed.vector.x * 3.6

    def callback_ego_acceleration(self, msg_accel: Vector3Stamped):
        self.ego_accel = msg_accel.vector.x

    def callback_ego_steering(self, msg_steer: Vector3Stamped):
        self.ego_steer = msg_steer.vector.z

    def callback_tm_action(self, msg_action: Vector3Stamped):
        action_flt = msg_action.vector.z
        
        self.tm_action = "none"

        if action_flt == 4.0:
            self.tm_action = 'LaneFollow'
        elif action_flt == 1.0:
            self.tm_action = 'Left'
        elif action_flt == 2.0:
            self.tm_action = 'Right'
        elif action_flt == 3.0:
            self.tm_action = 'Straight'        


    def _parse_vehicle_keys(self):

        keys = pygame.key.get_pressed()
        self.clock.tick()
        milliseconds = self.clock.get_time()
        self._notifications.tick(self.clock)
        
        pygame.event.get()

        if keys[K_UP] or keys[K_w]:
            self.throttle = min(self.throttle + 0.1, 1)
        else:
            self.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self.brake = min(self.brake + 0.2, 1)
        else:
            self.brake = 0.0

        self.acceleration = self.throttle-self.brake

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self.steering = round(self._steer_cache, 1)


        if keys[K_m]:
            self.mode = Mode.MANUAL
            self.get_logger().info("Mode set to MANUAL")
        elif keys[K_n]:
            self.mode = Mode.AUTO
            self.get_logger().info("Mode set to AUTO")
        elif keys[K_b]:
            self.mode = Mode.AI
            self.get_logger().info("Mode set to AI")
        
        # Quit if escape is pressed
        if keys[K_ESCAPE]:
            self.get_logger().info("Exiting")
            self.close_ui()
            sys.exit(0)
            

    def repaint(self):
        text = f"speed: {self.ego_speed:.2f}, acceleration: {self.ego_accel:.2f}, steering: {self.ego_steer:.2f}, action: {self.tm_action}, mode: {self.mode.name}"
        self._notifications.set_text(text)

        if self.rgb_frame is not None:
            self.screen.blit(self.rgb_frame, (0,0))
        if self.lidar_frame is not None:
            self.screen.blit(self.lidar_frame, (WIDTH,0))

        self._notifications.render(self.screen)
        pygame.display.flip()


    def __del__(self):
        self.close_ui()

    def timer_callback(self):

        self.repaint()

        self._parse_vehicle_keys()
        time_stamp = self.get_clock().now().to_msg()

        v_accel = Vector3Stamped()
        v_accel.header.stamp = time_stamp
        v_accel.vector = Vector3()
        v_accel.vector.x = float(self.acceleration)
        self.pub_accel.publish(v_accel)


        v_steer = Vector3Stamped()
        v_steer.header.stamp = time_stamp
        v_accel.vector = Vector3()
        v_steer.vector.z = float(self.steering)
        self.pub_steer.publish(v_steer)

        v_mode = Vector3Stamped()
        v_mode.header.stamp = time_stamp
        v_mode.vector = Vector3()
        v_mode.vector.x = float(self.mode.value)
        self.pub_mode.publish(v_mode)


    def callback_rgb_image(self, image):
        array = self.cv_bridge.compressed_imgmsg_to_cv2(image)
        array = array[:, :, ::-1]  # Convert BGR to RGB
        array = cv2.resize(array, (WIDTH, HEIGHT))
        self.rgb_frame = pygame.surfarray.make_surface(array.swapaxes(0, 1))


    def callback_lidar_data(self, lidar):

        array = np.zeros((HEIGHT, WIDTH, 3))
        points = np.frombuffer(lidar.data, dtype=np.dtype('f4'))
        carla_pointcloud = np.reshape(points, (int(points.shape[0] / 4), 4))
        carla_pointcloud = np.asarray(ZOOM_IN * carla_pointcloud[:, :2], dtype=int) # Zoom in and transform to integers for getting indexes
        pts_indexes = np.logical_and(np.logical_and(HEIGHT > carla_pointcloud[:,0], carla_pointcloud[:, 0] > 0), # check maximum front distance
        np.logical_and(WIDTH / 2 > carla_pointcloud[:, 1], carla_pointcloud[:, 1] > -WIDTH / 2)) # check maximum sides of image
        carla_pointcloud = carla_pointcloud[pts_indexes, :] # filter valid points
        carla_pointcloud[:, 1] = carla_pointcloud[:, 1] + int(WIDTH/2) # center pointcloud
        carla_pointcloud[:, 0] = HEIGHT - carla_pointcloud[:, 0] # inverse upside for visualization
        array[carla_pointcloud[:, 0], carla_pointcloud[:, 1], :] = 255 # draw lidar pointclouds
        array[HEIGHT-3:HEIGHT, int(WIDTH/2)-3:int(WIDTH/2)+3, :] = (255, 0, 0) # draw lidar sensor position in red

        self.lidar_frame = pygame.surfarray.make_surface(array.swapaxes(0, 1))



    ## function to initialise the window/events system
    def init_ui(self):
        self.get_logger().info("Initializing I/O System")
        # Initialize pygame
        pygame.init()
        pygame.font.init()

        self.get_logger().info("\tSetting up notifications system")
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (WIDTH, 40), (0, HEIGHT - 40))

        self.get_logger().info("\tSetting up clock")
        self.clock = pygame.time.Clock()

        # Set up the display
        self.get_logger().info("\tSetting up window")
        self.screen = pygame.display.set_mode((WIDTH*2, HEIGHT))
        pygame.display.set_caption("RGB Cam")

        # Create a surface with a simple design
        self.image = pygame.Surface((WIDTH*2, HEIGHT))  # Create a blank surface
        self.image.fill((50, 150, 250))  # Fill it with a color (RGB: blueish)

        # Draw a red rectangle
        pygame.draw.rect(self.image, (255, 0, 0), (200, 150, 400, 300))


    ## function to finish the window/events system
    def close_ui(self):
        self.get_logger().info("Fisihing I/O System")
        pygame.quit()



def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()