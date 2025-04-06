import pygame
import numpy as np
import threading
import queue
import os
from datetime import datetime
from time import sleep

import carla
import time
import numpy as np
import math
import pandas as pd
import cv2


HOST = "localhost"
PORT = 2300
TM_PORT = PORT + 6000
MAP_NAME = "Town10HD_Opt"
WEATHER = "ClearSunset"
EGO_VEHICLE= "vehicle.dodge.charger_2020"
ZOOM_IN = 6
WIDTH, HEIGHT = 1600, 600
OUTPUT_DIR = "./run_{}/".format(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))


def to_bgra_array(image):
    """Convert a CARLA raw image to a BGRA numpy array."""
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    return array

def to_rgb_array(image):
    """Convert a CARLA raw image to a RGB numpy array."""
    array = to_bgra_array(image)
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array

def lidar_to_numpy(carla_pointcloud):
    array = np.zeros((HEIGHT, WIDTH//2, 3))
    carla_pointcloud = np.asarray(ZOOM_IN * carla_pointcloud[:, :2], dtype=int) # Zoom in and transform to integers for getting indexes
    pts_indexes = np.logical_and(np.logical_and(HEIGHT > carla_pointcloud[:,0], carla_pointcloud[:, 0] > 0), # check maximum front distance
        np.logical_and((WIDTH//2) / 2 > carla_pointcloud[:, 1], carla_pointcloud[:,1] > -(WIDTH//2) / 2)) # check maximum sides of image
    carla_pointcloud = carla_pointcloud[pts_indexes, :] # filter valid points
    carla_pointcloud[:, 1] = carla_pointcloud[:, 1] + int((WIDTH//2)/2) # center pointcloud
    carla_pointcloud[:, 0] = HEIGHT - carla_pointcloud[:, 0] # inverse upside for visualization
    array[carla_pointcloud[:, 0], carla_pointcloud[:, 1], :] = 255 # draw lidar pointclouds
    array[HEIGHT-3:HEIGHT, int((WIDTH//2)/2)-3:int((WIDTH//2)/2)+3, :] = (255, 0, 0) # draw lidar sensor position in red
    return array

def get_image_pygame(image):
    """ Add CARLA image to pygame display and queue """
    image = to_rgb_array(image)
    image_queue.put(image)
    
    # Resizing the image to fit the display
    image = cv2.resize(image, (WIDTH//2, HEIGHT))
    # Converting the image to a pygame surface
    image = pygame.surfarray.make_surface(image.swapaxes(0, 1))

    screen.blit(image, (0, 0))


def get_lidar_pygame(lidar_points):
    """ Add CARLA lidar points to pygame display and queue """
    points = np.frombuffer(lidar_points.raw_data, dtype=np.dtype('f4')).reshape(-1, 4)
    image = lidar_to_numpy(points)
    lidar_queue.put(image)

    # Converting the image to a pygame surface
    image = pygame.surfarray.make_surface(image.swapaxes(0, 1))

    screen.blit(image, (WIDTH//2, 0))

def image_saver():
    """ Save images from the queue to disk """
    counter = 0 # counter to avoid saving too many images
    while counter < 200 and running:
        # rgb image
        try:
            image = image_queue.get(timeout=1)
            cv2.imwrite(os.path.join(OUTPUT_DIR, f"image_{counter:02d}.png"), image[..., ::-1])
            counter += 1
        except queue.Empty:
            continue
        
        # lidar image
        try:
            lidar = lidar_queue.get(timeout=1)
            cv2.imwrite(os.path.join(OUTPUT_DIR, f"lidar_{counter:02d}.png"), lidar[..., ::-1])
            counter += 1
        except queue.Empty:
            continue


print("Configuring the client")
client = carla.Client(HOST, PORT)
client.set_timeout(60.0)
client.load_world(MAP_NAME)

print("Setting world parameters")
sim_world = client.get_world()
settings = sim_world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
settings.max_substeps = 16
sim_world.apply_settings(settings)

print("Creating the Ego vehicle")
blueprint_library = sim_world.get_blueprint_library()
ego_bp = blueprint_library.find(EGO_VEHICLE)

trafic_manager = client.get_trafficmanager(TM_PORT)
trafic_manager.set_synchronous_mode(True)

spawn_points = sim_world.get_map().get_spawn_points()
spawn_point = spawn_points[np.random.randint(0, len(spawn_points))]

ego = sim_world.try_spawn_actor(ego_bp, spawn_point)

# Camera sensor
cam_bp = sim_world.get_blueprint_library().find('sensor.camera.rgb')
cam_bp.set_attribute("image_size_x",str(800))
cam_bp.set_attribute("image_size_y",str(600))
cam_bp.set_attribute("fov",str(105))

cam_location = carla.Location(x=1.7, y=0, z=1.25)
cam_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
cam_transform = carla.Transform(cam_location,cam_rotation)
ego_cam = sim_world.spawn_actor(cam_bp, cam_transform, attach_to=ego, attachment_type=carla.AttachmentType.Rigid)

# Lidar sensor
lidar_bp = sim_world.get_blueprint_library().find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels', str(1))
lidar_bp.set_attribute('lower_fov', str(-5))
lidar_bp.set_attribute('upper_fov', str(1))
lidar_bp.set_attribute('rotation_frequency', str(20))
lidar_bp.set_attribute('range', str(100))

lidar_location = carla.Location(x=2, y=0, z=1.25)
lidar_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
lidar_transform = carla.Transform(lidar_location,lidar_rotation)
ego_lidar = sim_world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego, attachment_type=carla.AttachmentType.Rigid)

# create a queue to store the images in the background
running = True
image_queue = queue.Queue()
lidar_queue = queue.Queue()

# Create the directory to save the images
os.makedirs(OUTPUT_DIR, exist_ok=True)

saving_thread = threading.Thread(target=image_saver)
saving_thread.start()

# Initialize pygame
print("Initializing pygame")
pygame.init()

# Set up the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("CARLA Simulation")

# Save the images each frame
ego_cam.listen(lambda image: get_image_pygame(image))
ego_lidar.listen(lambda lidar_points: get_lidar_pygame(lidar_points))

# Detting the autopilot mode
ego.set_autopilot(True, TM_PORT)

print("Running the simulation")
start_time = time.time()
last_time = start_time


my_dict = {"time": [], "throttle": [], "steer": [], "velocity": [], "indication": []}

# Main loop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

    # Simulation world tick
    sim_world.tick()

    # Update the display
    pygame.display.flip()

    # Getiing the neccessary data
    control = ego.get_control()
    velocity = ego.get_velocity()
    velocity_mod = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    indication = trafic_manager.get_next_action(ego)[0]

    my_dict["time"].append(time.time() - start_time)
    my_dict["throttle"].append(control.throttle)
    my_dict["steer"].append(control.steer)
    my_dict["velocity"].append(velocity_mod)
    my_dict["indication"].append(indication)

    time.sleep(0.01)

# Generating the csv file with the data
data = pd.DataFrame(my_dict)
data.to_csv(os.path.join(OUTPUT_DIR, "data.csv"), index=False)

saving_thread.join()
pygame.quit()

print("Program finished")
print("run saved in {}".format(OUTPUT_DIR))