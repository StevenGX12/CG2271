import pygame
import time
import socket
import numpy as np

TCP_IP = '192.168.1.1'
TCP_PORT = 2001
BUFFER_SIZE = 1024

last_payload = bytes([255, 127, 127, 0, 0, 0, 0, 0, 0, 0, 255])

pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((100,100))


# Initialize the joysticks.
pygame.joystick.init()

arcade_flag = False
last_arcade_toggle_btn_state = False

def get_servo_spd(joystick_axis, servo_x_spd):
    if joystick_axis < -0.7:
        servo_x_spd = 0
    elif joystick_axis < -0.4:
        servo_x_spd = 1
    elif joystick_axis < -0.1:
        servo_x_spd = 2
    elif joystick_axis < 0.1:
        servo_x_spd = 3
    elif joystick_axis < 0.4:
        servo_x_spd = 4
    elif joystick_axis < 0.7:
        servo_x_spd = 5
    else:
        servo_x_spd = 6
    return servo_x_spd

def map_inputs_to_buffer(xbox_leftjoy_horiz, 
                         xbox_leftjoy_vert, 
                         xbox_rightjoy_horiz, 
                         xbox_rightjoy_vert, 

                         usb_leftjoy_horiz, 
                         usb_leftjoy_vert, 
                         usb_rightjoy_horiz, 
                         usb_rightjoy_vert,
                         
                         xbox_arcade_toggle_btn = None,
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

    if xbox_arcade_toggle_btn is not None:
        if xbox_arcade_toggle_btn and last_arcade_toggle_btn_state == False:    # If toggled from low to high:
            arcade_flag = not arcade_flag                   # Invert the flag
            # print("ay")
        last_arcade_toggle_btn_state = xbox_arcade_toggle_btn    # Update state
    # print(arcade_toggle_btn)
    # print(last_arcade_toggle_btn_state)

    left_spd = 0    # Left Motor Speed. Ranges from -127 to +127. Must be >= min_motor_speed to have visable rotation.
    right_spd = 0   # Right Motor Speed. Ranges from -127 to +127. Must be >= min_motor_speed to have visable rotation.
    servo_1_spd = 3
    servo_2_spd = 3
    servo_3_spd = 3

    # Tank Drive Control
    if arcade_flag == False:
        abs_leftjoy_vert = np.abs(xbox_leftjoy_vert)
        if abs_leftjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
            sign = np.sign(xbox_leftjoy_vert)            # Store sign of the joystick value. 
            left_spd = sign*(min_motor_speed + (127 - min_motor_speed)*((abs_leftjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output

        abs_rightjoy_vert = np.abs(xbox_rightjoy_vert)
        if abs_rightjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
            sign = -1 * np.sign(xbox_rightjoy_vert)       # Store sign of the joystick value. Note that sign is flipped since positive is downwards by default
            right_spd = sign*(min_motor_speed + (127 - min_motor_speed)*((abs_rightjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output

    
    # Arcade Drive Control
    else:
        # Standard Arcade Conversion
        arcade_left_spd = xbox_leftjoy_vert - xbox_leftjoy_horiz
        arcade_right_spd = xbox_leftjoy_vert + xbox_leftjoy_horiz

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
        # if usb_rightjoy_vert < -0.7:
        #     servo_1_spd = 0
        # elif usb_rightjoy_vert < -0.4:
        #     servo_1_spd = 1
        # elif usb_rightjoy_vert < -0.1:
        #     servo_1_spd = 2
        # elif usb_rightjoy_vert < 0.1:
        #     servo_1_spd = 3
        # elif usb_rightjoy_vert < 0.4:
        #     servo_1_spd = 4
        # elif usb_rightjoy_vert < 0.7:
        #     servo_1_spd = 5
        # else:
        #     servo_1_spd = 6
        servo_1_spd = get_servo_spd(usb_rightjoy_vert, servo_1_spd)
        servo_2_spd = get_servo_spd(usb_leftjoy_vert, servo_2_spd)
        servo_3_spd = get_servo_spd(usb_rightjoy_horiz, servo_3_spd)
        # servo_4_spd = get_servo_spd(usb_leftjoy_horiz, servo_4_spd)

    shifted_left_spd = int(left_spd + 127)
    shifted_right_spd = int(right_spd + 127)

    payload = bytes([255, shifted_left_spd, shifted_right_spd, servo_1_spd, servo_2_spd, servo_3_spd, 255]) ## not all controls in use yet
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

        xbox_leftjoy_horiz = 0
        xbox_leftjoy_vert = 0
        xbox_rightjoy_horiz =0 
        xbox_rightjoy_vert =0
        xbox_arcade_toggle_btn = False

        usb_leftjoy_horiz = 0
        usb_leftjoy_vert =0 
        usb_rightjoy_horiz= 0
        usb_rightjoy_vert = 0
        # Localise code to XBOX ONE Controller
        if joystick_name=='Controller (Xbox One For Windows)':
            # XBOX ONE Controller Settings
            axes = joystick.get_numaxes()
            xbox_leftjoy_horiz = joystick.get_axis(0)
            xbox_leftjoy_vert = joystick.get_axis(1)
            xbox_rightjoy_horiz = joystick.get_axis(2)
            xbox_rightjoy_vert = joystick.get_axis(3)
            xbox_leftflip = joystick.get_axis(4)
            xbox_rightflip = joystick.get_axis(5)

            xbox_buttons = joystick.get_numbuttons()
            xbox_leftbump = joystick.get_button(4)
            xbox_rightbump = joystick.get_button(5)

            # output = map_inputs_to_buffer(leftjoy_horiz, leftjoy_vert, rightjoy_horiz, rightjoy_vert, arcade_toggle_btn = leftbump)
            # if last_payload != output:
            #     transmit_tcp(output)
            #     last_payload = output

        if joystick_name=='USB Gamepad':
            # USB Gamepad
            axes = joystick.get_numaxes()
            usb_leftjoy_horiz = joystick.get_axis(0)
            usb_leftjoy_vert = joystick.get_axis(1)
            usb_rightjoy_horiz = joystick.get_axis(3)
            usb_rightjoy_vert = joystick.get_axis(2)
            # output = map_inputs_to_buffer(usb_leftjoy_horiz, usb_leftjoy_vert, usb_rightjoy_horiz, usb_rightjoy_vert)
            # if last_payload != output:
            #     transmit_tcp(output)
            #     last_payload = output
           
        output = map_inputs_to_buffer(
            xbox_leftjoy_horiz, 
            xbox_leftjoy_vert, 
            xbox_rightjoy_horiz, 
            xbox_rightjoy_vert, 
            usb_leftjoy_horiz, 
            usb_leftjoy_vert, 
            usb_rightjoy_horiz, 
            usb_rightjoy_vert, 
            
            xbox_arcade_toggle_btn = xbox_leftbump)
            
        if last_payload != output:
            transmit_tcp(output)
            last_payload = output

    clock.tick(10)


# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()