from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from hub import BT_VCP
from math import *
import time

hub = PrimeHub()

hub.light_matrix.show_image('HAPPY')

def init():
    cat_motor = Motor('A')
    goose_motor = Motor('B')
    duck_motor = Motor('C')
    force_sensor = ForceSensor('D')
    color_sensor = ColorSensor('E')
    distance_sensor = DistanceSensor('F')
    motion_sensor = MotionSensor('G')
    motor_pair = MotorPair('A', 'B')
    # initialize remaining sensors
    # TODO: initialize MotorPair

class BT():
    def __init__(self):
        self.com  = BT_VCP(0)
        while not self.com.isconnected():
            hub.status_light.on('red')
        hub.status_light.off('red')
        hub.status_light.on('green')
        self.com.setinterrupt(-1) # TODO: functionality not clear
        
        def connection_status():
            if not self.com.isconnected():
                hub.status_light.off('green')
                hub.status_light.on('red')

        self.com.callback(connection_status())
    
    def connect(self):
        count = 0
        while not hub.left_button.is_pressed():
            received_bytes = self.com.recv(1024, timeout = 100)
            if not received_bytes: continue
            received_data = received_bytes.decode()
            print("Received: ", received_data)

            data_to_send = "Bye{}".format(count)
            send_bytes = self.com.send(data_to_send.encode('utf-8'), timeout=100)
            count += 1

            print("Send: ", data_to_send)

    def any(self): # what does this function do?   
        return self.com.any()

    def read(self, msg):
        if self.any():
            received_bytes = self.com.recv(1024, timeout=100)
            if received_bytes:
                msg.append(received_bytes)

    def close_connection(self):
        self.com.close()


# TODO: modify move class 
class Move(): # class to move the robot

    def __init__(self): # front_wheels = (right, left)
        self.motor = MotorPair('F', 'E')
        self.distance = DistanceSensor('B')
        self.motor.set_default_speed(50) # check if correct
    
    def check_obstacle(self)->bool: # Obstacle: True  No obstacle: False
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
        while hub.motion_sensor.get_yaw_angle() > -87:
            self.motor.start_tank(left_speed=-50, right_speed=50)
            time.sleep(0.1)
            self.motor.stop()
        # angle = hub.motion_sensor.get_yaw_angle()
        # print('Angle:', angle)

    def turn_right(self):
        hub.motion_sensor.reset_yaw_angle()
        while hub.motion_sensor.get_yaw_angle() < 87:
            self.motor.start_tank(left_speed=50, right_speed=-50)
            time.sleep(0.1)
            self.motor.stop()
        # angle = hub.motion_sensor.get_yaw_angle()
        # print('Angle:', angle)

    def turn_around(self):
        self.turn_right()
        self.turn_right()

    def avoid_obstacle(): # future upgrade
        pass

class FoodDispenser():
    
    def __init__(self):
        self.cat_motor = Motor('D')
        self.goose_motor = Motor('C')
        self.duck_motor = Motor('A')
    
    def serve_cat_food(self):
        self.cat_motor.run_for_rotations(0.5)
    
    def serve_goose_food(self):
        self.goose_motor.run_for_rotations(0.5)

    def serve_duck_food(self):
        self.duck_motor.run_for_rotations(0.5)


bt = BT()
food = FoodDispenser()

msg = []
while not hub.left_button.is_pressed():
    if bt.any():
        readinto(msg, 2)

        



    

    

