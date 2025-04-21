import carla
import cv2
import numpy as np
import time
import random
from detectors.yolov8_detector import YOLOv8Detector
from detectors.ssd_detector import SSDDetector
from detectors.hog_svm_detector import HOGSVMDetector
from utils.plotter import LivePlotter
from utils.gps_utils import load_gps_path, find_next_gps_point
import threading


client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

spawn_point = random.choice(world.get_map().get_spawn_points())
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
ego = world.spawn_actor(vehicle_bp, spawn_point)

#Add camera
cam_bp = blueprint_library.find('sensor.camera.rgb')
cam_bp.set_attribute("image_size_x", "640")
cam_bp.set_attribute("image_size_y", "480")
cam_bp.set_attribute("fov", "110")
cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(cam_bp, cam_transform, attach_to=ego)

#GPS sensor
gps_bp = blueprint_library.find('sensor.other.gnss')
gps_sensor = world.spawn_actor(gps_bp, carla.Transform(), attach_to=ego)

#Model
model_name = "YOLOv8"
if model_name == "YOLOv8":
    detector = YOLOv8Detector()
elif model_name == "SSD":
    detector = SSDDetector("models/MobileNetSSD_deploy.prototxt.txt", "models/MobileNetSSD_deploy.caffemodel")
else:
    detector = HOGSVMDetector()

#GPS Tracking
current_gps = {"lat": 0.0, "lon": 0.0}
gps_path = load_gps_path("gps_path.pkl")

def gps_callback(data):
    current_gps["lat"] = data.latitude
    current_gps["lon"] = data.longitude

gps_sensor.listen(gps_callback)

#Obstacle Detection
def check_front_obstacle(vehicle, world, distance=5.0):
    loc = vehicle.get_location()
    forward_vector = vehicle.get_transform().get_forward_vector()
    end = loc + forward_vector * distance
    raycast = world.cast_ray(loc, end)
    return len(raycast) > 0

# ==== Simple GPS-based control ====
def follow_waypoints(vehicle, world, throttle_val=0.3):
    current_loc = vehicle.get_location()
    waypoint = world.get_map().get_waypoint(current_loc).next(2.0)[0]
    steer = get_steer(vehicle.get_transform(), waypoint.transform)
    return carla.VehicleControl(throttle=throttle_val, steer=steer)

def get_steer(current_transform, target_transform):
    current_yaw = current_transform.rotation.yaw % 360
    target_yaw = target_transform.rotation.yaw % 360
    angle_diff = (target_yaw - current_yaw + 180) % 360 - 180
    return np.clip(angle_diff / 90.0, -1.0, 1.0)

#Plotting
plotter = LivePlotter()
plot_thread = threading.Thread(target=plotter.run, daemon=True)
plot_thread.start()

# Main Camera Callback
def image_callback(image):
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))[:, :, :3]
    frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    #detection
    output_frame, inference_time = detector.detect(frame)
    plotter.update(model_name, inference_time)


    cv2.imshow("Camera", output_frame)
    cv2.waitKey(1)

    #control
    if check_front_obstacle(ego, world):
        control = carla.VehicleControl(throttle=0.0, brake=1.0)
    else:
        target = find_next_gps_point(current_gps["lat"], current_gps["lon"], gps_path)
        if target:
            control = follow_waypoints(ego, world, throttle_val=0.4)
        else:
            control = carla.VehicleControl(throttle=0.0)
    ego.apply_control(control)

#Start Stream
camera.listen(image_callback)


try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    camera.stop()
    gps_sensor.stop()
    ego.destroy()
    cv2.destroyAllWindows()
