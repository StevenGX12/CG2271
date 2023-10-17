import pygame
import time
import socket
import numpy as np

TCP_IP = '192.168.1.1'
TCP_PORT = 2001
BUFFER_SIZE = 1024

pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((100,100))


# Initialize the joysticks.
pygame.joystick.init()

def map_inputs_to_buffer(leftjoy_horiz, leftjoy_vert, rightjoy_horiz, rightjoy_vert,
                         joystick_deadzone = 0.05, 
                         min_motor_speed = 32):
    # Inputs:
    #   1. Left Joystick (Horizontal Axis) value, ranging from -1 to 1 (Positive Right)
    #   2. Left Joystick (Vertical Axis) value, ranging from -1 to 1 (Positive Bottom)
    #   3. Right Joystick (Horizontal Axis) value, ranging from -1 to 1 (Positive Right)
    #   4. Right Joystick (Vertical Axis) value, ranging from -1 to 1 (Positive Bottom)
    # Outputs:
    #   1. Bytestring that can be sent to the robot

    ### motors ###
    left_spd = 0    # Left Motor Speed. Ranges from -127 to +127. Must be >= min_motor_speed to have visable rotation.
    right_spd = 0   # Right Motor Speed. Ranges from -127 to +127. Must be >= min_motor_speed to have visable rotation.

    abs_leftjoy_vert = np.abs(leftjoy_vert)
    if abs_leftjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
        sign = -1 * np.sign(leftjoy_vert)       # Store sign of the joystick value. Note that sign is flipped since positive is downwards by default
        left_spd = sign*(min_motor_speed + (127 - min_motor_speed)*((abs_leftjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output

    abs_rightjoy_vert = np.abs(rightjoy_vert)
    if abs_rightjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
        sign = -1 * np.sign(rightjoy_vert)       # Store sign of the joystick value. Note that sign is flipped since positive is downwards by default
        right_spd = sign*(min_motor_speed + (127 - min_motor_speed)*((abs_rightjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output

    shifted_left_spd = int(left_spd + 127)
    shifted_right_spd = int(right_spd + 127)
    print(f"{shifted_left_spd},{shifted_right_spd}")

    ### servos ###

    servo_1_pos = 0
    servo_2_pos = 0
    
    abs_leftjoy_horiz = np.abs(leftjoy_horiz)
    if abs_leftjoy_horiz > joystick_deadzone: 
        servo_1_pos = leftjoy_horiz

    abs_rightjoy_horiz = np.abs(rightjoy_horiz)
    if abs_rightjoy_horiz > joystick_deadzone: 
        servo_1_pos = rightjoy_horiz
    
    payload = bytes([255, shifted_left_spd, shifted_right_spd, servo_1_pos, servo_2_pos, 0, 255])
    return payload

def transmit_tcp(payload):
    try:
        s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP,TCP_PORT))
        s.send(payload)
        s.close()
        print(f"Tx: {payload}")
    except Exception as e:
        print(f"<Error> {e}")

# -------- Main Program Loop -----------
done = False
while not done: 
    # Window Closing Handler
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.

    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()

    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        # Get GUID (for joystick recognition)
        try:
            joystick_guid = joystick.get_guid()
        except AttributeError:
            # get_guid() is an SDL2 method
            pass
        
        # Get Name (for joystick recognitiion)
        joystick_name = joystick.get_name()
        
        # Localise code to XBOX ONE Controller
        if joystick_name=='Controller (Xbox One For Windows)':
            # XBOX ONE Controller Settings
            axes = joystick.get_numaxes()

            ### motor ###
            leftjoy_horiz = joystick.get_axis(0)
            leftjoy_vert = joystick.get_axis(1)
            rightjoy_horiz = joystick.get_axis(2)
            rightjoy_vert = joystick.get_axis(3)
            leftflip = joystick.get_axis(4)
            rightflip = joystick.get_axis(5)

            buttons = joystick.get_numbuttons()
            leftbump = joystick.get_button(4)
            rightbump = joystick.get_button(5)

          




            output = map_inputs_to_buffer(leftjoy_horiz, leftjoy_vert, rightjoy_horiz, rightjoy_vert)
            transmit_tcp(output)

    clock.tick(20)


# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()