import pygame
import time
import socket
import numpy as np

# -------- TCP Declaration --------
TCP_IP = '192.168.1.1'
TCP_PORT = 2001
BUFFER_SIZE = 1024

# -------- Constants --------
joystick_deadzone = 0.05
min_motor_speed = 32

# -------- Arcade Toggle Latch --------
controller_state = 0
last_arcade_toggle_btn_state = False

# -------- PyGame Text Printing Service --------
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')
class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

# -------- Pygame Startup Routine --------
pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((100,100))
pygame.joystick.init()

# -------- TCP Transmission Function --------
# def transmit_tcp(payload):
#     try:
#         s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         s.connect((TCP_IP,TCP_PORT))
#         s.send(payload)
#         s.close()
#         print(f"Tx: {payload}")
#     except Exception as e:
#         print(f"<Error> {e}")

# -------- Main Program Loop -----------
done = False
s = None

while not done: 
    # Window Closing Handler
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.


    # Initialise Buttons
    j1_left_horiz = None
    j1_left_vert = None
    j1_right_horiz = None
    j1_right_vert = None
    j1_left_bumper = None
    j1_right_bumper = None
    j1_left_trigger = None
    j1_right_trigger = None
    j1_btn_y = None
    j1_btn_a = None
    j1_btn_menu = None
    j1_hat = None

    j2_left_horiz = None
    j2_left_vert = None
    j2_right_horiz = None
    j2_right_vert = None
    j2_left_top_bumper = None
    j2_left_bot_bumper = None
    j2_right_top_bumper = None
    j2_right_bot_bumper = None

    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()

    # Assign joystick outputs
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        # # Get GUID (for joystick recognition)
        # try:
        #     joystick_guid = joystick.get_guid()
        # except AttributeError:
        #     # get_guid() is an SDL2 method
        #     pass
        
        # Get Name (for joystick recognitiion)
        joystick_name = joystick.get_name()

        # Localise code to XBOX ONE Controller
        if joystick_name=='Controller (Xbox One For Windows)':
            # XBOX ONE Controller Settings
            axes = joystick.get_numaxes()
            j1_left_horiz = joystick.get_axis(0)
            j1_left_vert = joystick.get_axis(1)
            j1_right_horiz = joystick.get_axis(2)
            j1_right_vert = joystick.get_axis(3)
            j1_left_trigger = joystick.get_axis(4)
            j1_right_trigger = joystick.get_axis(5)

            buttons = joystick.get_numbuttons()
            j1_left_bumper = joystick.get_button(4)
            j1_right_bumper = joystick.get_button(5)
            j1_btn_y = joystick.get_button(3)
            j1_btn_a = joystick.get_button(0)
            j1_btn_menu = joystick.get_button(7)
            j1_hat = joystick.get_hat(i)


        elif joystick_name=='USB Gamepad':
            # USB Gamepad
            axes = joystick.get_numaxes()
            j2_left_horiz = joystick.get_axis(0)
            j2_left_vert = joystick.get_axis(1)
            j2_right_horiz = joystick.get_axis(3)
            j2_right_vert = joystick.get_axis(2)
    
            buttons = joystick.get_numbuttons()
            j2_left_top_bumper = joystick.get_button(4)
            j2_left_bot_bumper = joystick.get_button(6)
            j2_right_top_bumper = joystick.get_button(5)
            j2_right_bot_bumper = joystick.get_button(7)

    # Controller State Tracker
    if j1_btn_menu is not None:
        if j1_btn_menu and last_arcade_toggle_btn_state == False:    # If toggled from low to high:
            if controller_state >= 1:
                controller_state = 0    # Recycle State
            else:
                controller_state += 1   # Increment State
        last_arcade_toggle_btn_state = j1_btn_menu    # Update memory 

    # Declare Payload Contents
    left_spd = 0        # Left Motor Speed. Ranges from -126 to +126. Must be >= min_motor_speed to have visable rotation.
    right_spd = 0       # Right Motor Speed. Ranges from -126 to +126. Must be >= min_motor_speed to have visable rotation.
    servo_1_spd = 3     # Servo states: Map discrete speeds of -3, -2, -1, [0], 1, 2, 3 to 0, 1, 2, [3], 4, 5, 6. If value is >10, map to servo angle
    servo_2_spd = 3
    servo_3_spd = 3
    winch_speed = 0

    # Execute control script based on controller state
    if controller_state == 0 and j1_left_horiz is not None:
        # ****************************** Execute Tank Drive ******************************
        abs_leftjoy_vert = np.abs(j1_left_vert)
        if abs_leftjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
            sign = np.sign(j1_left_vert)            # Store sign of the joystick value. 
            left_spd = sign*(min_motor_speed + (126 - min_motor_speed)*((abs_leftjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output

        abs_rightjoy_vert = np.abs(j1_right_vert)
        if abs_rightjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
            sign = -1 * np.sign(j1_right_vert)       # Store sign of the joystick value. Note that sign is flipped since positive is downwards by default
            right_spd = sign*(min_motor_speed + (126 - min_motor_speed)*((abs_rightjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output
        
        # Arm Axis 1 Control
        if j1_right_bumper:
            servo_1_spd = 6
        elif j1_right_trigger > 0.01:
            servo_1_spd = 0
        else:
            servo_1_spd = 3

        # Ramp Control
        if j1_btn_y:
            servo_2_spd = 6
        elif j1_btn_a:
            servo_2_spd = 0
        else:
            servo_2_spd = 3


        # Claw Control
        if j1_left_bumper:
            servo_3_spd = 6
        elif j1_left_trigger > 0.01:
            servo_3_spd = 0
        else:
            servo_3_spd = 3

        # Winch Control
        if j1_hat[1] == -1:
            winch_speed = -120
        elif j1_hat[1] == 1:
            winch_speed = 120
        else:
            winch_speed = 0


    elif controller_state == 1:
        # ****************************** Execute Arcade Drive ******************************
        pass

    shifted_left_spd = int(np.clip(left_spd + 126, 0, 253))
    shifted_right_spd = int(np.clip(right_spd + 126, 0, 253))
    shifted_winch_speed = int(np.clip(winch_speed + 126, 0, 253))

    payload = bytes([254, shifted_left_spd, shifted_right_spd, servo_1_spd, servo_2_spd, servo_3_spd, shifted_winch_speed, 255])
    # print(payload)
        
    try:
        s.send(payload)
        print(f"Tx: {payload}")
    except Exception as e:
        try:
            s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((TCP_IP,TCP_PORT))
            s.send(payload)
        except Exception as e:
            print(f"<Error> {e}")

    

    # transmit_tcp(payload)


    clock.tick(8)


# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()












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