import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================





import carla



spawn_point = carla.Transform(carla.Location(x=-2900, y=5.3, z=0.1), carla.Rotation(roll=0,pitch=0,yaw=90))

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(2000.0)

sim_world = client.get_world()




try:

    vehicle_bp = sim_world.get_blueprint_library().filter("vehicle.*")[0]    
    vehicle_actor = sim_world.try_spawn_actor(vehicle_bp, spawn_point)

    while True:
        sim_world.wait_for_tick()



finally:
    vehicle_actor.destroy()
