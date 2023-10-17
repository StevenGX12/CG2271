import bluetooth
import pygame
import time
import socket
import numpy as np

# -------- Constants --------
joystick_deadzone = 0.05
min_motor_speed = 32

# -------- Arcade Toggle Latch --------
controller_state = 0
last_arcade_toggle_up_btn_state = False
last_arcade_toggle_down_btn_state = False

# -------- Main Program Loop -----------
done = False
s = None

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

state_enumeration = [
    "Pick Up",
    "Bumpy Road",
    "Multi Slope",
    "Bowling/Tunnel/Golf/Seesaw/Sand/Water"
]

# -------- Pygame Startup Routine --------
print("Initialising PyGame...")
pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((400,400))
pygame.joystick.init()
textPrint = TextPrint()
print("PyGame Ready!")

# Set the MAC address of the Bluetooth device you want to connect to
target_address = "C0:49:EF:69:FE:82"

# Search for nearby Bluetooth devices
nearby_devices = bluetooth.discover_devices()

# Try to connect to the target device
print(f"Connecting to {target_address}...")
while True:
    if done:
        break
    try:
        # Create a Bluetooth socket and connect to the target device
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.connect((target_address, 1))

        print(f"Connection Established!")

        while not done: 
            
            # Reset Printer
            screen.fill(WHITE)
            textPrint.reset()

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
            j1_btn_screen = None
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
                    j1_btn_screen = joystick.get_button(6)
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
                if j1_btn_menu and last_arcade_toggle_up_btn_state == False:    # If toggled from low to high:
                    if controller_state >= 3:
                        controller_state = 0    # Recycle State
                    else:
                        controller_state += 1   # Increment State
                last_arcade_toggle_up_btn_state = j1_btn_menu    # Update memory 
            if j1_btn_screen is not None:
                if j1_btn_screen and last_arcade_toggle_down_btn_state == False:    # If toggled from low to high:
                    if controller_state <= 0:
                        controller_state = 3    # Recycle State
                    else:
                        controller_state -= 1   # Decrement State
                last_arcade_toggle_down_btn_state = j1_btn_screen    # Update memory 


            textPrint.tprint(screen, "Controller State: {}".format(state_enumeration[controller_state]))
            textPrint.indent()


            # Declare Payload Contents
            left_spd = 0        # Left Motor Speed. Ranges from -126 to +126. Must be >= min_motor_speed to have visable rotation.
            right_spd = 0       # Right Motor Speed. Ranges from -126 to +126. Must be >= min_motor_speed to have visable rotation.
            servo_1_spd = 3     # Servo states: Map discrete speeds of -3, -2, -1, [0], 1, 2, 3 to 0, 1, 2, [3], 4, 5, 6. If value is >10, map to servo angle
            servo_2_spd = 3
            servo_3_spd = 3
            servo_4_spd = 3
            winch_speed = 0

            # Execute control script based on controller state
            if controller_state == 0 and j1_left_horiz is not None:
                # ****************************** State 0: Pickup ******************************
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
                    servo_1_spd = 5
                elif j1_right_trigger > 0.01:
                    servo_1_spd = 1
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

                textPrint.tprint(screen, "Tank Drive; [L] Claw / [R] Arm / [Y] Ramp")
                textPrint.indent()


            elif controller_state == 1 and j1_left_horiz is not None:
                # ****************************** State 1: Bumpy Road ******************************
                abs_leftjoy_vert = np.abs(j1_left_vert)
                if abs_leftjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
                    sign = 1*np.sign(j1_left_vert)            # Store sign of the joystick value. 
                    right_spd = sign*(min_motor_speed + (126 - min_motor_speed)*((abs_leftjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output

                abs_rightjoy_vert = np.abs(j1_right_vert)
                if abs_rightjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
                    sign = -1 * np.sign(j1_right_vert)       # Store sign of the joystick value. Note that sign is flipped since positive is downwards by default
                    left_spd = sign*(min_motor_speed + (126 - min_motor_speed)*((abs_rightjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output
                
                # Arm Axis 1 Control
                if j1_right_bumper:
                    servo_1_spd = 6
                elif j1_right_trigger > 0.01:
                    servo_1_spd = 0
                else:
                    servo_1_spd = 3

                # Ramp Control
                if j1_left_bumper:
                    servo_2_spd = 6
                elif j1_left_trigger > 0.01:
                    servo_2_spd = 0
                else:
                    servo_2_spd = 3

                # Claw Control
                if j1_btn_y:
                    servo_3_spd = 6
                elif j1_btn_a:
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

                textPrint.tprint(screen, "Inverted Tank Drive; [L] Ramp / [R] Arm / [Y] Claw")
                textPrint.indent()


            elif controller_state == 2 and j1_left_horiz is not None:
                # ****************************** State 2: Multi Slope ******************************
                abs_leftjoy_vert = np.abs(j1_left_vert)
                if abs_leftjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
                    sign = np.sign(j1_left_vert)            # Store sign of the joystick value. 
                    left_spd = sign*(min_motor_speed + (126 - min_motor_speed)*((abs_leftjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output

                abs_rightjoy_vert = np.abs(j1_right_vert)
                if abs_rightjoy_vert > joystick_deadzone:    # Check that joystick is out of deadzone
                    sign = -1 * np.sign(j1_right_vert)       # Store sign of the joystick value. Note that sign is flipped since positive is downwards by default
                    right_spd = sign*(min_motor_speed + (126 - min_motor_speed)*((abs_rightjoy_vert - joystick_deadzone)/(1-joystick_deadzone)))  # Scale output
                
                if j1_right_bumper:
                    servo_1_spd = 5
                elif j1_right_trigger > 0.01:
                    servo_1_spd = 1 
                else:
                    servo_1_spd = 3

                if j1_left_bumper:
                    winch_speed = -120
                    print("A")
                elif j1_left_trigger > 0.01:
                    winch_speed = 120
                    print("B")
                else:
                    winch_speed = 0

                # Claw Control
                if j1_btn_y:
                    servo_2_spd = 6
                elif j1_btn_a:
                    servo_2_spd = 0
                else:
                    servo_2_spd = 3

                textPrint.tprint(screen, "Tank Drive; [L] Winch / [R] Arm / [Y] Ramp")
                textPrint.indent()

            elif controller_state == 3 and j1_left_horiz is not None:
                # ****************************** State 3: Bowling/Tunnel/Golf/Onwards ******************************
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
                if j1_left_bumper:
                    servo_2_spd = 6
                elif j1_left_trigger > 0.01:
                    servo_2_spd = 0
                else:
                    servo_2_spd = 3

                # Claw Control
                if j1_btn_y:
                    servo_3_spd = 6
                elif j1_btn_a:
                    servo_3_spd = 0
                else:
                    servo_3_spd = 3
                textPrint.tprint(screen, "Tank Drive; [L] Ramp / [R] Arm / [Y] Kicker")
                textPrint.indent()

                # Support Servo Mechanism
                if j1_hat[1] == -1:
                    servo_4_spd = 6
                elif j1_hat[1] == 1:
                    servo_4_spd = 0
                else:
                    servo_4_spd = 3




            shifted_left_spd = int(np.clip(left_spd + 126, 0, 253))
            shifted_right_spd = int(np.clip(right_spd + 126, 0, 253))
            shifted_winch_speed = int(np.clip(winch_speed + 126, 0, 253))

            payload = bytes([254, shifted_left_spd, shifted_right_spd, servo_1_spd, servo_2_spd, servo_3_spd, shifted_winch_speed, servo_4_spd, 255])
            sock.send(payload)
            print(f"Tx: {payload}")
            pygame.display.flip()
            clock.tick(8)

        # # Receive some data from the target device
        # data = sock.recv(1024)
        # print(f"Received: {data}")

        # Close the socket
        sock.close()

    except:
        # Failed to connect to the target device
        print(f"Connection Failure! <MAC> {target_address}\nReattempting Connection...")
        pass

print("<ENDTASK>")

