import pygame
import time
import socket
import numpy as np

TCP_IP = '192.168.1.1'
TCP_PORT = 2001
BUFFER_SIZE = 1024

pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((300,300))

# Initialize the joysticks.
pygame.joystick.init()









########################### OLD BELOW #####################################

last_payload = bytes([255, 127, 127, 0, 0, 0, 255])


arcade_flag = False
last_arcade_toggle_btn_state = False

def map_inputs_to_buffer(leftjoy_horiz, leftjoy_vert, rightjoy_horiz, rightjoy_vert, 
                         arcade_toggle_btn = None,
                         joystick_deadzone = 0.05, 
                         min_motor_speed = 32):
    # Inputs:
    #   1. Left Joystick (Horizontal Axis) value, ranging from -1 to 1 (Positive Right)
    #   2. Left Joystick (Vertical Axis) value, ranging from -1 to 1 (Positive Bottom)
    #   3. Right Joystick (Horizontal Axis) value, ranging from -1 to 1 (Positive Right)
    #   4. Right Joystick (Vertical Axis) value, ranging from -1 to 1 (Positive Bottom)
    # Outputs:
    #   1. Bytestring that can be sent to the robot
    global arcade_flag, last_arcade_toggle_btn_state

    if arcade_toggle_btn is not None:
        if arcade_toggle_btn and last_arcade_toggle_btn_state == False:    # If toggled from low to high:
            arcade_flag = not arcade_flag                   # Invert the flag
            # print("ay")
        last_arcade_toggle_btn_state = arcade_toggle_btn    # Update state
    # print(arcade_toggle_btn)
    # print(last_arcade_toggle_btn_state)

    left_spd = 0    # Left Motor Speed. Ranges from -127 to +127. Must be >= min_motor_speed to have visable rotation.
    right_spd = 0   # Right Motor Speed. Ranges from -127 to +127. Must be >= min_motor_speed to have visable rotation.
    servo_1_spd = 3

    # Tank Drive Control
    if arcade_flag == False:
        abs_leftjoy_vert = np.abs(leftjoy_vert)
        if abs_leftjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
            sign = np.sign(leftjoy_vert)            # Store sign of the joystick value. 
            left_spd = sign*(min_motor_speed + (127 - min_motor_speed)*((abs_leftjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output

        abs_rightjoy_vert = np.abs(rightjoy_vert)
        if abs_rightjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
            sign = -1 * np.sign(rightjoy_vert)       # Store sign of the joystick value. Note that sign is flipped since positive is downwards by default
            right_spd = sign*(min_motor_speed + (127 - min_motor_speed)*((abs_rightjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output

    
    # Arcade Drive Control
    else:
        # Standard Arcade Conversion
        arcade_left_spd = leftjoy_vert - leftjoy_horiz
        arcade_right_spd = leftjoy_vert + leftjoy_horiz

        # Crmip to [-1, 1]
        if arcade_left_spd > 1:     
            arcade_left_spd = 1
        elif arcade_left_spd < -1:
            arcade_left_spd = -1
        if arcade_right_spd > 1:     
            arcade_right_spd = 1
        elif arcade_right_spd < -1:
            arcade_right_spd = -1

        # Get Absolutes
        abs_arcade_left_speed = np.abs(arcade_left_spd)
        abs_arcade_right_speed = np.abs(arcade_right_spd)

        # Deadzone Checking
        arcade_deadzone = joystick_deadzone*2
        if abs_arcade_left_speed > arcade_deadzone:
            sign = 1 * np.sign(arcade_left_spd)
            left_spd = sign * (min_motor_speed + (127 - min_motor_speed) * ((abs_arcade_left_speed - arcade_deadzone)/(1-arcade_deadzone))) # Scale Output
        if abs_arcade_right_speed > arcade_deadzone:
            sign = -1 * np.sign(arcade_right_spd)
            right_spd = sign * (min_motor_speed + (127 - min_motor_speed) * ((abs_arcade_right_speed - arcade_deadzone)/(1-arcade_deadzone))) # Scale Output
        
        # Arm Control
        if rightjoy_vert < -0.7:
            servo_1_spd = 0
        elif rightjoy_vert < -0.4:
            servo_1_spd = 1
        elif rightjoy_vert < -0.1:
            servo_1_spd = 2
        elif rightjoy_vert < 0.1:
            servo_1_spd = 3
        elif rightjoy_vert < 0.4:
            servo_1_spd = 4
        elif rightjoy_vert < 0.7:
            servo_1_spd = 5
        else:
            servo_1_spd = 6


    shifted_left_spd = int(left_spd + 127)
    shifted_right_spd = int(right_spd + 127)

    payload = bytes([255, shifted_left_spd, shifted_right_spd, servo_1_spd, 0, 0, 255])
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
            leftjoy_horiz = joystick.get_axis(0)
            leftjoy_vert = joystick.get_axis(1)
            rightjoy_horiz = joystick.get_axis(2)
            rightjoy_vert = joystick.get_axis(3)
            leftflip = joystick.get_axis(4)
            rightflip = joystick.get_axis(5)

            buttons = joystick.get_numbuttons()
            leftbump = joystick.get_button(4)
            rightbump = joystick.get_button(5)

            output = map_inputs_to_buffer(leftjoy_horiz, leftjoy_vert, rightjoy_horiz, rightjoy_vert, arcade_toggle_btn = leftbump)
            if last_payload != output:
                transmit_tcp(output)
                last_payload = output

        elif joystick_name=='USB Gamepad':
            # USB Gamepad
            axes = joystick.get_numaxes()
            leftjoy_horiz = joystick.get_axis(0)
            leftjoy_vert = joystick.get_axis(1)
            rightjoy_horiz = joystick.get_axis(3)
            rightjoy_vert = joystick.get_axis(2)
            output = map_inputs_to_buffer(leftjoy_horiz, leftjoy_vert, rightjoy_horiz, rightjoy_vert)
            if last_payload != output:
                transmit_tcp(output)
                last_payload = output

    clock.tick(10)


# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()