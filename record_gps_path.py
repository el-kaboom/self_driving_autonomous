import carla
import pickle
import time

gps_path = []

def save_gps_callback(data):
    gps_path.append((data.latitude, data.longitude))
    print(f"📍 GPS: {data.latitude:.6f}, {data.longitude:.6f}")

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()

vehicle_bp = bp_lib.filter("model3")[0]
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True)

gps_bp = bp_lib.find('sensor.other.gnss')
gps_sensor = world.spawn_actor(gps_bp, carla.Transform(), attach_to=vehicle)
gps_sensor.listen(save_gps_callback)

try:
    print("Recording GPS path... Drive around for ~20-30s")
    time.sleep(30)  # record for 30 seconds
finally:
    gps_sensor.stop()
    vehicle.destroy()
    with open("gps_path.pkl", "wb") as f:
        pickle.dump(gps_path, f)
    print("Saved GPS path to gps_path.pkl")