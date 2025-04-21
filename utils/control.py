import carla
import numpy as np

def check_front_obstacle(vehicle, world, distance=5.0):
    loc = vehicle.get_location()
    forward = vehicle.get_transform().get_forward_vector()
    start = loc + carla.Location(z=1.0)
    end = start + forward * distance

    raycast = world.cast_ray(start, end)
    for hit in raycast:
        if hit.actor.id != vehicle.id:
            return True
    return False

def follow_waypoints(vehicle, world, throttle_val=0.4):
    current = vehicle.get_location()
    waypoint = world.get_map().get_waypoint(current)
    next_wp = waypoint.next(5.0)[0]

    vehicle_transform = vehicle.get_transform()
    target_loc = next_wp.transform.location

    yaw = vehicle_transform.rotation.yaw
    target_yaw = next_wp.transform.rotation.yaw
    steer_angle = (target_yaw - yaw) / 180.0

    control = carla.VehicleControl()
    control.throttle = throttle_val
    control.steer = float(np.clip(steer_angle, -1.0, 1.0))
    return control
