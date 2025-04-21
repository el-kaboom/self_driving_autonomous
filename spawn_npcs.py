import carla
import random
import time

def spawn_npcs(client, num_vehicles=30, num_pedestrians=40):
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    spawn_points = world.get_map().get_spawn_points()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    for _ in range(num_vehicles):
        bp = random.choice(bp_lib.filter("vehicle"))
        transform = random.choice(spawn_points)
        try:
            npc = world.spawn_actor(bp, transform)
            npc.set_autopilot(True)
        except:
            pass

    walker_bp = bp_lib.filter('walker.pedestrian.*')
    controller_bp = bp_lib.find('controller.ai.walker')

    for _ in range(num_pedestrians):
        spawn_loc = world.get_random_location_from_navigation()
        if spawn_loc:
            ped = world.try_spawn_actor(random.choice(walker_bp), carla.Transform(spawn_loc))
            if ped:
                controller = world.spawn_actor(controller_bp, carla.Transform(), attach_to=ped)
                controller.start()
                controller.go_to_location(world.get_random_location_from_navigation())
                controller.set_max_speed(1.4)

if __name__ == "__main__":
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    client.load_world('Town03')
    spawn_npcs(client)
