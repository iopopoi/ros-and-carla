import glob
from logging import addLevelName
import os
import re
import sys
from typing import ClassVar

from numpy.lib.function_base import append, disp

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

import argparse
import math
# import collections
# import datetime
import random
import time
import weakref
# import logging

try:
    import pygame 
    #     from pygame.locals import KMOD_CTRL
    #     from pygame.locals import KMOD_SHIFT
    #     from pygame.locals import K_0
    #     from pygame.locals import K_9
    #     from pygame.locals import K_BACKQUOTE
    #     from pygame.locals import K_BACKSPACE
    #     from pygame.locals import K_COMMA
    #     from pygame.locals import K_DOWN
    #     from pygame.locals import K_ESCAPE
    #     from pygame.locals import K_F1
    #     from pygame.locals import K_LEFT
    #     from pygame.locals import K_PERIOD
    #     from pygame.locals import K_RIGHT
    #     from pygame.locals import K_SLASH
    #     from pygame.locals import K_SPACE
    #     from pygame.locals import K_TAB
    #     from pygame.locals import K_UP
    #     from pygame.locals import K_a
    #     from pygame.locals import K_c
    #     from pygame.locals import K_g
    #     from pygame.locals import K_d
    #     from pygame.locals import K_h
    #     from pygame.locals import K_m
    #     from pygame.locals import K_n
    #     from pygame.locals import K_p
    #     from pygame.locals import K_q
    #     from pygame.locals import K_r
    #     from pygame.locals import K_s
    #     from pygame.locals import K_w
    #     from pygame.locals import K_l
    #     from pygame.locals import K_i
    #     from pygame.locals import K_z
    #     from pygame.locals import K_x
    #     from pygame.locals import K_MINUS
    #     from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# WORLD
class World(object):
    def __init__(self, carla_world, args, client):
        self.world = carla_world
        self.client = client
        
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

        self.blueprint_library = self.world.get_blueprint_library()
        self.world_snapshot = None

        self.sensor_camera = None
        self.sensor_imu = None
        self.sensor_gnss = None
        self.sensor_lidar = None
        self.actor_appended = None

        self.actor_filter = 'vehicle.audi.a2'

        self.gamma = args.gamma
        self.width_x = args.width
        self.width_y = args.height

        self.set_weather(args)
        self.start()

    def start(self):
        # append walkers and vehicles

        self.actor_appended = ManageActor(self.world, self.client)

        # vehicle - player
        blueprint = random.choice(self.blueprint_library.filter(self.actor_filter))
        blueprint.set_attribute('role_name', 'simulator - vehicle')

        transform = random.choice(self.world.get_map().get_spawn_points())
    
        self.player = self.world.spawn_actor(blueprint, transform)
        print('APPEND Player %s' % self.player.type_id)
        self.player.set_autopilot(True)        

        # bounding box for traffic_light
        # self.debugger()

        # set sensors
        self.sensor_camera = ManageCamera(self.player, self.gamma, self.width_x, self.width_y)
        self.sensor_lidar = ManageLidar(self.player,self.width_x, self.width_y)
        self.sensor_imu = ManageImu(self.player)
        self.sensor_gnss = ManageGnss(self.player)

    # set weather
    def set_weather(self, args):
        weather = carla.WeatherParameters(
            cloudiness= args.cloud,
            precipitation = args.rain,
            precipitation_deposits = 0.0,
            sun_altitude_angle=70.0)
        self.world.set_weather(weather)

    # only for player (chenge traffic light to green) 
    def manage_traffic(self):
        if self.player.is_at_traffic_light():
            traffic_light = self.player.get_traffic_light()
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                traffic_light.set_state(carla.TrafficLightState.Green)
    
    def render(self, display):
        self.sensor_camera.render(display)
        # self.sensor_lidar.render(display)

    # destroy sensors and actors
    def destroy(self):

        actors = [
            self.sensor_camera.sensor,
            self.sensor_gnss.sensor,
            self.sensor_imu.sensor,
            self.sensor_lidar.sensor,
            self.player
        ]

        for actor in actors:
            if actor is not None:
                print("destroy: %s" % actor)
                actor.destroy()
        print("destroy vehicles & actors")
        self.actor_appended.destroy()


    # bounding box for bounding_list(ex: traffic light)
    def debugger(self):

        debug = self.world.debug
        self.world_snapshot = self.world.get_snapshot()

        for actor_snapshot in self.world_snapshot:
            actual_actor = self.world.get_actor(actor_snapshot.id)
            # bounding list
            bounding_list = ['traffic.traffic_light']
            if actual_actor.type_id in bounding_list:
                debug.draw_box(carla.BoundingBox(actor_snapshot.get_transform().location,
                                carla.Vector3D(0.5,0.5,2)),actor_snapshot.get_transform().rotation, 0.05, 
                                carla.Color(255,0,0,0),0)

# KEY
class KeyControl(object):
    def __init__(self, world):
        self.autopilot_mode = True
        if isinstance(world.player, carla.Vehicle):
            self.control = carla.VehicleControl()
            world.player.set_autopilot(self.autopilot_mode)
        elif isinstance(world.player, carla.Walker):
            self.control = carla.WalkerControl()
            self.autopilot_mode = False
            self.rotation = world.player.get_transform().rotation

    def parse_events(self, client, world, clock):
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
    
# rgb camera
class ManageCamera(object):
    def __init__(self, parent_actor, gamma_parameter, width, height):
        self.sensor = None
        self.surface = None
        self.parent = parent_actor

        self.transform = carla.Transform(carla.Location(x=1, z=1.5))

        world = self.parent.get_world()
        blueprint_library = world.get_blueprint_library()

        # get bp of 'rgb camera'
        self.bp = blueprint_library.find('sensor.camera.rgb')
        self.bp.set_attribute('image_size_x', str(width))
        self.bp.set_attribute('image_size_y', str(height))
        self.bp.set_attribute('fov', str(90))

        if self.bp.has_attribute('gamma'):
            self.bp.set_attribute('gamma', str(gamma_parameter))

        # print("-- attrinbute of sensor: camera ------------")
        # for attr in self.bp:
        #     print(' - {}'.format(attr))

        self.set_sensor()

    def set_sensor(self):
        if self.sensor is not None:
            self.sensor.destroy()
            self.surface = None
        self.sensor = self.parent.get_world().spawn_actor(self.bp, self.transform, attach_to = self.parent)
        
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: ManageCamera.parse_image(weak_self, image))

    # render display
    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0,0))

    @staticmethod
    def parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        
        image.convert(cc.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        image.save_to_disk('output/%8d' % image.frame)

# Lidar
class ManageLidar(object):
    def __init__(self, player, width, height):
        self.sensor = None
        self.surface = None
        self.parent = player
        self.width = width
        self.height = height
        self.transform = carla.Transform(carla.Location(x=0, z=2.0))

        world = self.parent.get_world()
        blueprint_library = world.get_blueprint_library()

        self.bp = blueprint_library.find('sensor.lidar.ray_cast')
        self.bp.set_attribute('range', str(50))

        # print("[ Lidar ]========================")
        # for attr in self.bp:
        #     print(' - {}'.format(attr))

        self.set_sensor()

    def set_sensor(self):
        if self.sensor is not None:
            self.sensor.destroy()
            self.surface = None
        self.sensor = self.parent.get_world().spawn_actor(self.bp, self.transform, attach_to = self.parent)
        
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: ManageLidar.parse_image(weak_self, image))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0,0))

    @staticmethod
    def parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(self.width, self.height) / 100.0
        lidar_data += (0.5 * self.width, 0.5 * self.height)
        lidar_data = np.fabs(lidar_data)  
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (self.width, self.height, 3)
        lidar_img = np.zeros((lidar_img_size), dtype = int)
        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
        self.surface = pygame.surfarray.make_surface(lidar_img)

        # image.save_to_disk('output/%8d' % image.frame)

#Imu
class ManageImu(object):
    def __init__(self, player):
        self.sensor = None
        self.parent = player
        self.accel = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0

        world = self.parent.get_world()
        self.bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            self.bp, carla.Transform(), attach_to=self.parent)

        # print("[ IMU ]========================")
        # for attr in self.bp:
        #     print(' - {}'.format(attr))

        weak_self = weakref.ref(self)
        self.sensor.listen( lambda data: ManageImu.callback(weak_self, data) )

    @staticmethod
    def callback(weak_self, data):
        self = weak_self()
        if not self:
            return
        self.accel = ( data.accelerometer.x, data.accelerometer.y, data.accelerometer.z)
        self.gyroscope = ( math.degrees(data.gyroscope.x), math.degrees(data.gyroscope.y), math.degrees(data.gyroscope.z))
        self.compass = math.degrees(data.compass)
        
        # print(self.accel)
        # print(self.gyroscope)
        # print(self.compass)

#Gnss
class ManageGnss(object):
    def __init__(self, player):
        self.sensor = None 
        self.parent = player

        self.lat = 0.0
        self.lon = 0.0

        world = self.parent.get_world()
        self.bp = world.get_blueprint_library().find('sensor.other.gnss')

        self.sensor = world.spawn_actor(self.bp, carla.Transform(carla.Location(z=1)), attach_to=self.parent)

        # print("[ GNSS ]========================")
        # print(self.bp.get_attribute(self.sensor))

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda data: ManageGnss.callback(weak_self,data))

    @staticmethod
    def callback(weak_self, data):
        self = weak_self()
        if not self:
            return
        self.lat = data.latitude
        self.lon = data.longitude

        # print(self.lat)
        # print(self.lon)

# spawn additional walkers and vehicle
class ManageActor(object):
    def __init__(self, world, client):
        
        print("---[INIT ManageActor]------------")
        self.walker_list = []
        self.vehicle_list = []
        
        self.all_id = []
        self.all_actors = []

        self.world = world
        self.client = client

        print("set filter")
        # filter - vehicle and walker
        filter_vehicle = 'vehicle.*'
        self.num_vehicle = 10
        filter_walker = 'walker.pedestrian.*'
        self.num_walker = 20
        
        print("set traffic manager")
        self.traffic_manager = self.client.get_trafficmanager(8000)
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.0)

        self.synch = False

        # self.settings = self.world.get_settings()
        # self.traffic_manager.set_synchronous_mode(True)

        # if not self.settings.synchronous_mode:
        #     self.synch = True
        #     self.settings.synchronous_mode = True
        #     self.settings.fixed_delta_seconds = 0.05
        #     self.world.apply_settings(self.settings)
        # else:
        #     self.synch = False

        self.bp_vehicle = self.world.get_blueprint_library().filter(filter_vehicle)
        self.bp_walker = self.world.get_blueprint_library().filter(filter_walker)

        print("START SPAWN")
        self.spawn_vehicle()
        self.spawn_walker()
    
    def spawn_vehicle(self):
        print("-- spawn vehicles -----")
        # set vehicle
        self.bp_vehicle = [x for x in self.bp_vehicle if int(x.get_attribute('number_of_wheels')) == 4]
                # self.bp_vehicle = [x for x in self.bp_vehicle if not x.id.endswith('isetta')]
                # self.bp_vehicle = [x for x in self.bp_vehicle if not x.id.endswith('carlacola')]
                # self.bp_vehicle = [x for x in self.bp_vehicle if not x.id.endswith('cybertruck')]
                # self.bp_vehicle = [x for x in self.bp_vehicle if not x.id.endswith('t2')]

        spawn_points = self.world.get_map().get_spawn_points()
        num_spawn_points = len(spawn_points)

        if self.num_vehicle < num_spawn_points:
            random.shuffle(spawn_points)
        elif self.num_vehicle > num_spawn_points:
            self.num_vehicle = num_spawn_points

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        # make variable vehicle
        for i, transform in enumerate(spawn_points):
            if i >= self.num_vehicle:
                break

            bp = random.choice(self.bp_vehicle)

            # random color
            if bp.has_attribute('color'):
                color = random.choice(bp.get_attribute('color').recommended_values)
                bp.set_attribute('color', color)
            if bp.has_attribute('driver_id'):
                # recommended_value : A list of values suggested by those who designed the blueprint.
                driver_id = random.choice(bp.get_attribute('driver_id').recommended_values)
                bp.set_attribute('driver_id', driver_id)

            # set autopilot
            bp.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(bp, transform).then(SetAutopilot(FutureActor, True)))

        for res in self.client.apply_batch_sync(batch, self.synch):
            if res.error:
                print(res.error)
            else:
                self.vehicle_list.append(res.actor_id)

    def spawn_walker(self):
        print("-- spawn walkers -----")
        per_running = 0.0
        per_crossing = 0.0

        spawn_points = []
        SpawnActor = carla.command.SpawnActor

        for i in range(self.num_walker):
            point = carla.Transform()
            location = self.world.get_random_location_from_navigation()

            if location != None:
                point.location = location
                spawn_points.append(point)

        print("spawn walker")
        # spawn walker
        batch = []
        speed = []
        for point in spawn_points:
            bp = random.choice(self.bp_walker)
            if bp.has_attribute('is_invincible'):
                bp.set_attribute('is_invincible', 'false')
            if bp.has_attribute('speed'):
                if (random.random() > per_running): # walking
                    speed.append(bp.get_attribute('speed').recommended_values[1])
                else: # running
                    speed.append(bp.get_attribute('speed').recommended_values[2])
            else:               
                # print("walker has no spped: 0.0")
                speed.append(0.0)
            batch.append(SpawnActor(bp, point))
        
        results = self.client.apply_batch_sync(batch, True)

        speed2 = []
        for i in range(len(results)):
            if results[i].error:
                print(results[i].error)
            else:
                print("spawn success: %s" % results[i])
                self.walker_list.append({"id": results[i].actor_id})
                speed2.append(speed[i])
        speed = speed2
        
        print("spawn controller")
        # spawn walkwr controller
        batch = []
        con_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(self.walker_list)):
            batch.append(SpawnActor(con_bp, carla.Transform(), self.walker_list[i]["id"]))
        results = self.client.apply_batch_sync(batch, True)

        for i in range(len(results)):
            if results[i].error:
                # print(results[i].error)
                continue
            else:
                self.walker_list[i]["con"] = results[i].actor_id

        print("altogether walkers and controllers")
        #  altogether walkers and controllers
        for i in range(len(self.walker_list)):
            self.all_id.append(self.walker_list[i]["con"])
            self.all_id.append(self.walker_list[i]["id"])
        self.all_actors = self.world.get_actors(self.all_id)

        # wait for tick
        if not self.synch :
            self.world.wait_for_tick()
        else:
            self.world.tick()

        print("initialize each controller and set target")
        # initialize each controller and set target
        self.world.set_pedestrians_cross_factor(per_crossing)
        for i in range(0,len(self.all_id),2):
            self.all_actors[i].start()
            self.all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            self.all_actors[i].set_max_speed(float(speed[int(i/2)]))
        
        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(self.vehicle_list), len(self.walker_list)))

        # self.traffic_manager.global_percentge_speed_difference(30.0)

    def destroy(self):
        # if self.synch :
        #     self.settings = self.world.get_settings()
        #     self.settings.synchronous_mode = False
        #     self.settings.fixed_delta_seconds = None
        #     self.world.apply_settings(self.settings)

        print('\n=[DESTROY] %d vehicles' % len(self.vehicle_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicle_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(self.all_id), 2):
            self.all_actors[i].stop()

        print('\n=[DESTROY] %d walkers' % len(self.walker_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])


def game_loop(args):
    pygame.init()
    pygame.font.init()  

    try:

        #-------------------------
        # Client
        #-------------------------
        client = carla.Client('localhost',2000)
        client.set_timeout(2.0)
        print("END Client")


        #-------------------------
        # display
        #-------------------------
        display = pygame.display.set_mode(
            (args.width,args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)


        #-------------------------
        # world
        #-------------------------
        print(client.get_available_maps())
        client.load_world(args.town)

        print("[world]")
        world = World(client.get_world(), args, client)
        print("controller")
        controller = KeyControl(world)

        print("END world")
        
        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return
            world.manage_traffic()
            world.render(display)
            pygame.display.flip()

    finally:

        print('DESTROY ACTORS')
        # camera.destroy()
        if world is not None:
            print("destroy")
            world.destroy()
        pygame.quit()


def main():

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

    # window size default
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')

    # gamma
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')

    # town: choose town
    argparser.add_argument(
        '--town',
        default="Town01",
        help='choose map/town (default: Town01)')

    # rain 
    argparser.add_argument(
        '--rain',
        default=0.0,
        type=float,
        help='Rain intensity values range from 0 to 100')
    
    # cloud
    argparser.add_argument(
        '--cloud',
        default=0.0,
        type=float,
        help='cloudiness : values range from 0 to 100')


    # [ parse_args() ]
    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':

    main()
