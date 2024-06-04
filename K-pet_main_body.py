from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from hub import BT_VCP
from math import *
import time

hub = PrimeHub()

hub.light_matrix.show_image('HAPPY')

animal = -1


# message setting:
# 1. 2 byte for animal type
# 2. 6 bytes for position

class FoodDispenser():

    def __init__(self):
        self.cat_motor = Motor('A')
        self.duck_motor = Motor('C')
        self.goose_motor = Motor('D')
        self.cat_motor.run_to_position(degrees=90)
        self.cat_motor.run_to_position(degrees=90)
        self.goose_motor.run_to_position(degrees=90)
        self.goose_motor.run_to_position(degrees=90)
        self.duck_motor.run_to_position(degrees=0)
        self.duck_motor.run_to_position(degrees=0)

    def serve_cat_food(self):
        global hub
        try:
            self.cat_motor.run_for_rotations(0.5)
        except Exception as e:
            print('Error: ', e)
            hub.light_matrix.show_image('SAD')
            # self.cat_motor.run_to_position(degrees=90)


    def serve_goose_food(self):
        global hub
        try:
            self.goose_motor.run_for_rotations(0.5)
        except Exception as e:
            print('Error: ', e)
            hub.light_matrix.show_image('SAD')


    def serve_duck_food(self):
        global hub
        try:
            self.duck_motor.run_for_rotations(0.5)
        except Exception as e:
            print('Error: ', e)
            hub.light_matrix.show_image('SAD')


class Bluetooth():

    def connection_status():
        if not self.com.isconnected():
            hub.status_light.off('green')
            hub.status_light.on('red')

    def __init__(self):
        print('hello')
        self.com= BT_VCP(0)
        while not self.com.isconnected():
            hub.status_light.on('red')
        hub.status_light.on('green')
        time.sleep(3)
        hub.status_light.off()
        # self.com.setinterrupt(-1) # TODO: functionality not clear

        # self.com.callback() #self.connection_status()

    def connect(self):
        while not hub.left_button.is_pressed():
            received_bytes= self.com.readline()
            if not received_bytes != None: continue
            received_data = received_bytes.decode()
            print("Received: ", received_data)
            animal = int(received_data)
            return animal

        return -1

    def any(self): # what does this function do?
        return self.com.any()

    def read(self, msg):
        pass

    def close_connection(self):
        self.com.close()

class Move(): # class to move the robot

    def __init__(self): # front_wheels = (right, left)
        self.motor = MotorPair('F', 'E')
        self.distance = DistanceSensor('B')
        self.motor.set_default_speed(-20) # check if correct

    def check_obstacle(self)->bool: # Obstacle: TrueNo obstacle: False
        obstacle_distance = self.distance.get_distance_cm()
        if obstacle_distance != None and obstacle_distance < 30:
            return True
        return False

    def move_straight(self, distance):
        #self.motor.move_tank(amount = distance , unit='cm', left_speed=-50, right_speed=-50) # check if direction is correct
        self.motor.start()
        count = 0
        while not(self.check_obstacle()) and count < distance:
            count += 1
            time.sleep(1)
        self.motor.stop()


    def turn_left(self):
        hub.motion_sensor.reset_yaw_angle()
        while hub.motion_sensor.get_yaw_angle() > 87:
            self.motor.start_tank(left_speed=20, right_speed=-20)
            time.sleep(0.1)
            self.motor.stop()
        # angle = hub.motion_sensor.get_yaw_angle()
        # print('Angle:', angle)

    def turn_right(self):
        hub.motion_sensor.reset_yaw_angle()
        while hub.motion_sensor.get_yaw_angle() < 87:
            self.motor.start_tank(left_speed=-20, right_speed=20)
            time.sleep(0.1)
            self.motor.stop()
        # angle = hub.motion_sensor.get_yaw_angle()
        # print('Angle:', angle)

    def turn_around(self):
        self.turn_right()
        self.turn_right()

    def realign_to_direction(self, original_direction):
        while abs(hub.motion_sensor.get_yaw_angle() - original_direction) > 2:
            current_yaw = hub.motion_sensor.get_yaw_angle()
            if current_yaw < original_direction:
                self.motor.start_tank(left_speed=20, right_speed=-20)
            else:
                self.motor.start_tank(left_speed=-20, right_speed=20)
            time.sleep(0.1)
        self.motor.stop()

    def avoid_obstacle(self, original_direction): # future upgrade
        self.motor.stop()
        self.turn_left()
        self.motor.start()
        time.sleep(1)
        if self.check_obstacle():
            self.motor.stop()
            self.turn_right()
            self.motor.start()
            time.sleep(1)
            if self.check_obstacle():
                self.motor.stop()
                self.turn_around()
                self.motor.start()
                time.sleep(1)
                if self.check_obstacle():
                    self.motor.stop()
                    self.turn_right()
                    self.motor.start()
                    time.sleep(1)
                    if self.check_obstacle():
                        self.motor.stop()
                        self.turn_around()
                        self.motor.start()
                        time.sleep(1)
                        if self.check_obstacle():
                            self.motor.stop()
                            self.turn_left()
                            self.motor.start()
                            time.sleep(1)
        self.realign_to_direction(original_direction)

    def move_to_animal(self, distance):
        self.motor.start()
        count = 0
        hub.motion_sensor.reset_yaw_angle()
        original_direction = hub.motion_sensor.get_yaw_angle()
        while count < distance:
            if self.check_obstacle():
                self.avoid_obstacle((original_direction))
            else:
                self.motor.start()
                time.sleep(1)
                count += 1
        self.motor.stop()


bt = Bluetooth()
food = FoodDispenser()
movement = Move()

hub.left_button.wait_until_pressed()
hub.speaker.beep(seconds=1)
while True:
    msg = bt.connect()
    if msg >= 0 and msg <3:
        if msg == 0:
            # reach animal first
            # movement.move_to_animal(10)
            food.serve_cat_food()
            print('serving cat food')
        elif msg == 1:
            # reach animal first
            # movement.move_to_animal(10)
            food.serve_duck_food()
            print('serving duck food')
        elif msg == 2:
            # reach animal first
            # movement.move_to_animal(10)
            food.serve_goose_food()
            print('serving goose food')
        time.sleep(3)
    elif msg >= 3 or msg < 0:
        hub.light_matrix.show_image('SAD')
        break

'''

msg = []
while not hub.left_button.is_pressed():
    if bt.any():
        readinto(msg, 2)
'''








