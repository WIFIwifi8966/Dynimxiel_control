import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
from dynamixel_sdk import * # Uses Dynamixel SDK library    
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_GOAL_VELOCITY          = 104
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_VECLOCITY      = 128
DXL_MINIMUM_POSITION_VALUE  = 1023
ADDR_MODE = 11
DXL_MAXIMUM_POSITION_VALUE = 0      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600

PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = 'COM9'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE] 
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 2

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = 'COM9'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
portHandler.setBaudRate(BAUDRATE)
import pygame
 
# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)
 
# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)
 
    def print(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
        
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15
        
    def indent(self):
        self.x += 10
        
    def unindent(self):
        self.x -= 10
    
 
pygame.init()
 
# Set the width and height of the screen [width,height]
size = [500, 700]
screen = pygame.display.set_mode(size)
 
pygame.display.set_caption("Motor Controler")
 
#Loop until the user clicks the close button.
done = False
 
# Used to manage how fast the screen updates
clock = pygame.time.Clock()
 
# Initialize the joysticks
pygame.joystick.init()
    
# Get ready to print
textPrint = TextPrint()
x = 0
y = 0
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, 84, 128)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, 82, 8)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, 80, 128)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, 84, 128)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, 82, 8)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, 80, 128)

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, 11, 1)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 2, 11, 4)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 3, 11, 4)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, 44, 1023)
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, 12, 255)
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, 13, 2)
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, 13, 0)


dxl_present_positionx, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 2, ADDR_PRESENT_POSITION)
dxl_present_positiony, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 3, ADDR_PRESENT_POSITION)
y = dxl_present_positiony
x = dxl_present_positionx

    
# -------- Main Program Loop -----------
while done==False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
            
 
    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()
 
    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()
 
    textPrint.print(screen, "Number of joysticks: {}".format(joystick_count) )
    textPrint.indent()
    V = 100
    V_ID1 = 0
    
    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
    
        textPrint.print(screen, "Joystick {}".format(i) )
        textPrint.indent()
    
        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.print(screen, "Joystick name: {}".format(name) )
        
        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        textPrint.print(screen, "Number of axes: {}".format(axes) )
        textPrint.indent()
        axis = joystick.get_axis(0)
        textPrint.print(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
        if abs(axis) < 0.2:
            axis = 0
        elif axis > 0:
            axis = axis - 0.2
        else:
            axis = axis + 0.2
        y = V*axis + y    
        axis = joystick.get_axis(1)
        textPrint.print(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
        if abs(axis) < 0.2:
            axis = 0
        elif axis > 0:
            axis = axis - 0.2
        else:
            axis = axis + 0.2
        x = V*axis + x
        axis = joystick.get_axis(3)
        if abs(axis) < 0.2:
            axis = 0
        elif axis > 0:
            axis = axis - 0.2
        else:
            axis = axis + 0.2
        V_ID1 = 1250*axis
        textPrint.print(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
        # for i in range( axes ):
        #     V_ID1 = 0.0
        #     axis = joystick.get_axis( i )
        #     if abs(axis) < 0.2:
        #         axis = 0
        #     elif axis > 0:
        #         axis = axis - 0.2
        #     else:
        #         axis = axis + 0.2
        #     if i == 1:
        #         x = V*axis + x
        #     if i == 0:
        #         y = V*axis + y
        #     if i == 3:
        #         V_ID1 = 1250*axis
        
        textPrint.unindent()
    if x < 0:
        x = 0
    if y <0:
        y = 0
    if y > 2046:
        y = 2046
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, int(x))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, int(y))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, int(V_ID1))
    dxl_present_positionx, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 2, ADDR_PRESENT_POSITION)
    dxl_present_positiony, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 3, ADDR_PRESENT_POSITION)
    dxl_present_V0, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 1, ADDR_PRESENT_VECLOCITY)
    # KD, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, 2, 80)
    # KI, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, 2, 82)
    # KP, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, 2, 84)
    #print("[KD:%03d] [KI:%03d] [KP:%03d]"% (KD,KI,KP) )
    print("[ID:%03d] GoalPos:%03d  PresPos:%03d [ID:%03d] GoalPos:%03d  PresPos:%03d [ID:%03d] GoalVec:%03d PresVec:%03d"  % (3, y, dxl_present_positiony, 2, x, dxl_present_positionx,1,int(V_ID1),dxl_present_V0))
    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
 
    # Limit to 20 frames per second
    clock.tick(100)
    
# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit ()
# t1.join()