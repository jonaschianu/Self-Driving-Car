"""**************************************************************************************
*  Project:     Longitude & Lateral controllers for Waypoint tracking of vehicle        *
*                                                                                       *
*  Files:       waypoint_generator.py                                                   *
*  Description:  The file is used to generate waypoints text file to be used by         *
*                main.py file. The waypoints text file has location x, y and            *
*                velocity of vehicle . To generate waypoints, user can either manually  *
*                drive vehicle around the city or use autopilot. Key ‘r’ is used to     *
*                start/stop waypoint recording. Key ‘p’ is used to toggle autopilot     *
*                enable/disable. At every tick, vehicle location and velocity is        *
*                queried and this data is appended to waypoint.txt file                 *
***************************************************************************************



Use ARROWS or WASD keys for control.

    W /UP           : throttle
    S /Down         : brake
    A/D Left/Right  : steer left/right
    Q               : toggle reverse
    Space           : hand-brake
    P               : toggle autopilot
    R               : toggle recording images to disk
    ESC             : quit
"""
from __future__ import print_function

#************************************ NAMESPACE**************************************#

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

import carla
from carla import ColorConverter as cc
import math
import random
import weakref

try:
    import pygame

    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w

except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


#************************************ Class Definition**************************************#

class World:
    def __init__(self, carla_world, hud):
        """==================================================================================
        * Function:   __init__(): Initializes values
        * Arguments:  self, carla_world, hud
        * Returns:    N/A
        ==================================================================================="""
        self.world = carla_world
        try:
            # Exceptions handling for available maps
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.vehicle = None
        self.camera_manager = None
        self._gamma = 2.2
        self.start()
        # start n=method call to setup vehicle, sensor

    def start(self):
        """===================================================================================
        * Function:   start(): spawns vehicle, creates cameramanager object
        * Arguments:  N/A
        * Returns:    N/A
        ==================================================================================="""
        car_blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.tesla.model3'))
        car_blueprint.set_attribute('role_name', 'hero')
        # Selected vehicle is Tesla model 3
        while self.vehicle is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            # find spawn points for particular map
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            # select random spawn point
            self.vehicle = self.world.try_spawn_actor(car_blueprint, spawn_point)
            # Spawn vahicle
            self.waypoint_generate = Waypoint(self.vehicle)
            # Creates waypoint class object

        self.camera_manager = CameraManager(self.vehicle, self._gamma, self.hud)
        # Creates camermanager class object
        self.camera_manager.transform_index = 0
        self.camera_manager.set_sensor()
        # Sets up rgb camera sensor to vehicle spawned

    def tick(self, clock, key_control):
        """===================================================================================
        * Function:   tick(): Calls method to update hud variables
        * Arguments:  clock, key_control
        * Returns:    N/A
        ==================================================================================="""
        self.hud.tick(self, clock, key_control)

    def render(self, display):
        """===================================================================================
        * Function:   render(): renders frame of cameramanger and hud object
        * Arguments:  display
        * Returns:    N/A
        ==================================================================================="""
        self.camera_manager.render(display)
        self.hud.render(display)

    def append_waypoint(self):
        """===================================================================================
        * Function:   calls waypoint method to write data to text file
        * Arguments:  N/A
        * Returns:    N/A
        ==================================================================================="""
        self.waypoint_generate.print_waypoint()

    def destroy(self):
        sensors = [self.camera_manager.sensor]
        for sensor in sensors:
            # destroy  every sensor
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.vehicle is not None:
            # destroy spanwed vehicle
            self.vehicle.destroy()

#************************************ Class Definition**************************************#

class KeyboardControl:
    """Class that handles keyboard input."""

    def __init__(self, start_in_autopilot):
        """==================================================================================
        * Function:   __init__(): Initializes variables
        * Arguments:  self, start_in_autopilot
        * Returns:    N/A
        ==================================================================================="""
        self.autopilot_enabled = start_in_autopilot
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        self.waypoint_recording = False

    def parse_events(self, world, clock, client):
        """===================================================================================
        * Function:   parse_events(): key press event handler
        * Arguments:  world, clock
        * Returns:    N/A
        ==================================================================================="""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # user closed window, stops program
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    # If Esc key is pressed, stop program
                    return True
                elif event.key == K_r:
                    # toggle waypoint recording
                    self.waypoint_recording = not self.waypoint_recording
                elif event.key == K_q:
                    # toggle forward/reverse gear
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_p:
                    # toggle autopilot
                    self.autopilot_enabled = not self.autopilot_enabled
                    world.vehicle.set_autopilot(self.autopilot_enabled)
                    traffic_manager = client.get_trafficmanager()
                    traffic_manager.global_percentage_speed_difference(-100)
                    # vehicle in autopilot mode is not restricted to local speed limit
                    traffic_manager.ignore_lights_percentage(world.vehicle, 100)
                    # vehicle doesn't stop for red traffic lights at all time

        if not self.autopilot_enabled:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            # If autopilot is diabled figure out user input to control vehicle
            self._control.reverse = self._control.gear < 0
            world.vehicle.apply_control(self._control)
            # vehicle is set to manual control

    def _parse_vehicle_keys(self, keys, milliseconds):
        """===================================================================================
        * Function:   _parse_vehicle_keys(): sets vehicle control based on key press time
        * Arguments:  keys, milliseconds
        * Returns:    N/A
        ==================================================================================="""
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1)
            # detect throttle key and increase throtlle based on keys press duration
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            # detect brake key and increase brake value based on keys press duration
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            # detect left steer key and increase steer value based on time
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            # detect right steer key and increase steer value based on time
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]
        # detect handbrake keystroke and enable handbrake of vehicle

    @staticmethod
    def _is_quit_shortcut(key):
        return key == K_ESCAPE

#************************************ Class definition **************************************#

class SimulationDisplay:
    def render(self, display):
        """==================================================================================
        * Function:   render(): displays frame in pygame window
        * Arguments:  display
        * Returns:    N/A
        ==================================================================================="""
        display.blit(self.surface, (0, 0))

#************************************ Class definition **************************************#

class HUD(SimulationDisplay):
    def __init__(self, width, height):
        """==================================================================================
        * Function:   __init__(): Initializes width, height, font
        * Arguments:  width, font
        * Returns:    N/A
        ==================================================================================="""
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._info_text = []

    def tick(self, world, clock, key_control):
        """===================================================================================
        * Function:   tick(): refresh hud information variables
        * Arguments:  world, clock, key_control
        * Returns:    N/A
        ==================================================================================="""
        velocity = world.vehicle.get_velocity()
        # get current velocity of vehicle
        control = world.vehicle.get_control()
        #  get vehicle control variables
        self._info_text = [

            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Map:     % 20s' % world.map.name,
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2)),
            'Autopilot:%19s ' % key_control.autopilot_enabled,
            'Waypoint Recording:%10s ' % key_control.waypoint_recording,
            '']

        self._info_text += [
            ('Throttle:', control.throttle, 0.0, 1.0),
            ('Steer:', control.steer, -1.0, 1.0),
            ('Brake:', control.brake, 0.0, 1.0),
            ('Reverse:', control.reverse),
            ('Hand brake:', control.hand_brake)]

    def render(self, display):
        """===================================================================================
        * Function:   render(): renders HUD display
        * Arguments:  display
        * Returns:    N/A
        ==================================================================================="""
        info_surface = pygame.Surface((220, self.dim[1]))
        info_surface.set_alpha(100)
        display.blit(info_surface, (0, 0))
        v_offset = 4
        bar_h_offset = 100
        bar_width = 106
        for item in self._info_text:
            if v_offset + 18 > self.dim[1]:
                break
            if isinstance(item, list):
                if len(item) > 1:
                    points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                    pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                item = None
                v_offset += 18
            elif isinstance(item, tuple):
                if isinstance(item[1], bool):
                    rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                    pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                else:
                    rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                    pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                    f = (item[1] - item[2]) / (item[3] - item[2])
                    if item[2] < 0.0:
                        rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                    else:
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                    pygame.draw.rect(display, (255, 255, 255), rect)
                item = item[0]
            if item:  # At this point has to be a str.
                surface = self._font_mono.render(item, True, (255, 255, 255))
                display.blit(surface, (8, v_offset))
            v_offset += 18

#************************************ Class definition **************************************#

class CameraManager(SimulationDisplay):
    def __init__(self, parent_actor, gamma_correction, hud):
        """==================================================================================
        * Function:   __init__(): Initializes sensor, surface, hud
        * Arguments:  parent_actor, gamma_correction, hud
        * Returns:    N/A
        ==================================================================================="""
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.sensors = [['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}]]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        # get blueprint library
        for item in self.sensors:
            sensor_blueprint = bp_library.find(item[0])
            # find blueprint of rgb camera sensor
            if item[0].startswith('sensor.camera'):
                sensor_blueprint.set_attribute('image_size_x', str(1280))
                sensor_blueprint.set_attribute('image_size_y', str(720))
                # set sensor image dimensions
                if sensor_blueprint.has_attribute('gamma'):
                    sensor_blueprint.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    sensor_blueprint.set_attribute(attr_name, attr_value)
            item.append(sensor_blueprint)
            # add sensor to the list of item

    def set_sensor(self):
        """===================================================================================
        * Function:   set_sensor(): spawns rgb sensor and attaches to vehicle
        * Arguments:  N/A
        * Returns:    N/A
        ==================================================================================="""
        self.sensor = self._parent.get_world().spawn_actor(
            self.sensors[0][-1],
            carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)),
            attach_to=self._parent,
            attachment_type=carla.AttachmentType.SpringArm)
        # attach sensor to vehicle at given location and rotation
        weak_self = weakref.ref(self)
        # We need to pass the lambda a weak reference to self to avoid circular reference.
        self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))

    def render(self, display):
        """===================================================================================
        * Function:   render(): renders rgb camera sensor video
        * Arguments:  display
        * Returns:    N/A
        ==================================================================================="""
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(cc.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))



#************************************ Class definition **************************************#

class Waypoint:
    def __init__(self, vehicle):
        """==================================================================================
        * Function:   __init__(): Initializes vehicle, text_file, location, velocity
        * Arguments:  vehicle
        * Returns:    N/A
        ==================================================================================="""
        self.vehicle = vehicle
        self.text_file = None
        self.location = None
        self.velocity = None
        self.create_textfile()
        # Creates textfile and opens it in append mode

    def create_textfile(self):
        """===================================================================================
        * Function:   create_textfile(): creates textfile to load waypoints
        * Arguments:  display
        * Returns:    N/A
        ==================================================================================="""
        self.text_file = open(f"waypoint_carla.txt", "w")
        # creates new file if not present and overwites text file if file exists
        self.text_file.close()
        # close text file
        self.text_file = open(f"waypoint_carla.txt", "a")
        # open text file in append mode

    def print_waypoint(self):
        """===================================================================================
        * Function:   print_waypoint(): writes waypoints to text file
        * Arguments:  N/A
        * Returns:    N/A
        ==================================================================================="""
        self.location = self.vehicle.get_location()
        # get location of vehicle
        self.velocity = self.vehicle.get_velocity()
        # get velocity of vehciale
        velocity_magnitude = math.sqrt(self.velocity.x * self.velocity.x + self.velocity.y * self.velocity.y)
        # compute velocity magnitude of x and y direction velocity
        self.text_file.write(f"{self.location.x}, {self.location.y}, {velocity_magnitude}\n")
        # wirte waypoints to text file


#************************************ function definition **************************************#

def main():
    print(__doc__)
    display_width = 1280
    display_height = 720
    frame_per_second = 10
    try:
        pygame.init()
        pygame.font.init()
        world = None
        try:
            client = carla.Client('127.0.0.1', 2000) \
                # Configure client to local host and 2000 Tcp port
            client.set_timeout(5.0)
            # Set timeout of 5 sec

            display = pygame.display.set_mode(
                (display_width, display_height),
                pygame.HWSURFACE | pygame.DOUBLEBUF)
            # initializes the surface of given dimensions

            hud = HUD(display_width, display_height)
            # creates HUD class object
            world = World(client.get_world(), hud)
            # creates world class object
            key_control = KeyboardControl(False)

            clock = pygame.time.Clock()
            # creates pygame cock object to track time
            while True:
                clock.tick_busy_loop(frame_per_second)
                # restricts simulation step time
                if key_control.parse_events(world, clock, client):
                    # detects key press by user
                    return
                world.tick(clock, key_control)
                # Refresh the HUD display variables
                world.render(display)
                # renders new frame of camera sensor and HUD display
                if key_control.waypoint_recording is True:
                    world.append_waypoint()
                    # append waypoint data to text file
                pygame.display.flip()
                # updates display surafce on screen

        finally:
            if world is not None:
                world.destroy()

            pygame.quit()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
