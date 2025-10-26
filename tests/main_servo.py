import serial
import time
import numpy as np
from dynamixel_sdk import * # Uses Dynamixel SDK for servo control

# --- Main Configuration ---
IMU_PORT = "COM19"       # IMU serial port (adjust as needed)
IMU_BAUDRATE = 230400    # IMU baudrate

SERVO_PORT = 'COM14'     # Dynamixel servo serial port (adjust as needed)
SERVO_BAUDRATE = 115200  # Dynamixel baudrate
PROTOCOL_VERSION = 2.0

# --- Dynamixel Servo Control Constants (from init_servo_position.m) ---
# Control Table Addresses
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Data Values
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
POSITION_CONTROL_MODE = 3
EXT_POSITION_CONTROL_MODE = 4         # Operating Mode = 4 (Multi-turn)

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095
DXL_MOVING_STATUS_THRESHOLD = 20
MAX_SAFE_TURNS              = 50;                 # 最大安全圈数
MAX_POSITION_VALUE_extend   = 1048575;
MIN_POSITION_VALUE_extend   = -1048575;
UNITS_PER_DEG   = 4096.0 / 360.0
MULTI_TURN_GAIN = 8.0                 # 放大系数：角度 * 8，再映射为多圈（按需调大或调小）
# Servo IDs
DXL_ID_Z = 2 # Corresponds to the first servo in the MATLAB code
DXL_ID_X = 3 # Corresponds to the second servo in the MATLAB code

class IMUState:
    """
    A class to hold the state and data from the IMU, replacing MATLAB's global variables.
    """
    def __init__(self):
        self.buf = bytearray(5 + 73) # Rx packet buffer
        self.CS = 0
        self.i = 0
        self.RxIndex = 0
        self.cmdLen = 0
        self.quat_init = None # Initial quaternion [w, x, y, z]
        self.euler_init = None # Initial euler angles [roll, pitch, yaw]
        self.current_quat = [1.0, 0.0, 0.0, 0.0]
        self.current_euler = [0.0, 0.0, 0.0]
        self.is_initialized = False

        self.servo_init_pos_z = 0
        self.servo_init_pos_x = 0

# Global state instance
imu_state = IMUState()

def read_i16_le(buf, idx):
    # 小端、有符号16位
    v = (buf[idx+1] << 8) | buf[idx]
    if v >= 0x8000:
        v -= 0x10000
    return v, idx + 2


def cmd_pack_and_tx(ser, pDat):
    """
    Packs data into a command packet and sends it via serial.
    Equivalent to Cmd_PackAndTx from imupy.py.
    """
    DLen = len(pDat)
    if DLen == 0 or DLen > 19:
        print("Invalid data length")
        return -1

    # Build the packet buffer
    buf = bytearray([0x49, 0xFF, DLen]) + bytearray(pDat)
    CS = sum(buf[1:]) & 0xFF  # Checksum from address to end of data
    buf.append(CS)
    buf.append(0x4D)  # End byte

    ser.write(buf)
    return 0

def cmd_rx_unpack(buf):

    scaleAccel = 0.00478515625
    scaleQuat  = 0.000030517578125
    scaleAngle = 0.0054931640625

    if buf[0] != 0x11:
        print("------data head not define")
        return

    ctl = (buf[2] << 8) | buf[1]
    L = 7

    # Quaternion
    if (ctl & 0x0020):
        w, L = read_i16_le(buf, L); w *= scaleQuat
        x, L = read_i16_le(buf, L); x *= scaleQuat
        y, L = read_i16_le(buf, L); y *= scaleQuat
        z, L = read_i16_le(buf, L); z *= scaleQuat
        imu_state.current_quat = [w, x, y, z]

    # Euler
    if (ctl & 0x0040):
        roll,  L = read_i16_le(buf, L);  roll  *= scaleAngle
        pitch, L = read_i16_le(buf, L);  pitch *= scaleAngle
        yaw,   L = read_i16_le(buf, L);  yaw   *= scaleAngle
        imu_state.current_euler = [roll, pitch, yaw]

    if not imu_state.is_initialized and (ctl & 0x0020) and (ctl & 0x0040):
        imu_state.quat_init  = imu_state.current_quat
        imu_state.euler_init = imu_state.current_euler
        imu_state.is_initialized = True
        print("IMU Initialized:")
        print(f"  Initial Quaternion: {imu_state.quat_init}")
        print(f"  Initial Euler Angles: {imu_state.euler_init}")


def cmd_get_pkt(byte):
    """
    Processes one byte at a time to assemble a complete data packet.
    Equivalent to Cmd_GetPkt from imupy.py, using the imu_state object.
    """
    imu_state.CS += byte
    if imu_state.RxIndex == 0:  # Start of packet
        if byte == 0x49:
            imu_state.buf[0] = 0x49
            imu_state.i = 1
            imu_state.CS = 0  # Checksum starts from next byte
            imu_state.RxIndex = 1
    elif imu_state.RxIndex == 1:  # Address
        imu_state.buf[imu_state.i] = byte
        imu_state.i += 1
        imu_state.RxIndex = 2
    elif imu_state.RxIndex == 2:  # Length
        imu_state.buf[imu_state.i] = byte
        imu_state.i += 1
        if byte > 73 or byte == 0:  # Invalid length
            imu_state.RxIndex = 0
        else:
            imu_state.cmdLen = byte
            imu_state.RxIndex = 3
    elif imu_state.RxIndex == 3:  # Data payload
        imu_state.buf[imu_state.i] = byte
        imu_state.i += 1
        if imu_state.i >= imu_state.cmdLen + 3:
            imu_state.RxIndex = 4
    elif imu_state.RxIndex == 4:  # Checksum
        imu_state.CS -= byte
        if (imu_state.CS & 0xFF) == byte:  # Checksum is correct
            imu_state.buf[imu_state.i] = byte
            imu_state.i += 1
            imu_state.RxIndex = 5
        else:  # Checksum failed
            imu_state.RxIndex = 0
    elif imu_state.RxIndex == 5:  # End of packet
        imu_state.RxIndex = 0
        if byte == 0x4D:
            # Complete packet received, process it
            cmd_rx_unpack(imu_state.buf[3:imu_state.i-1])
            return 1
    else:
        imu_state.RxIndex = 0
    return 0

def configure_imu(ser):
    """
    Sends configuration parameters to the IMU.
    Equivalent to the setup part of read_data() in imupy.py.
    """
    print("Configuring IMU...")
    params = [0] * 11
    isCompassOn = 0
    barometerFilter = 2
    # Subscribe to Quaternion (0x20) and Euler angles (0x40)
    Cmd_ReportTag = 0x0020 | 0x0040 # 0x0FFF to get all data
    
    params[0] = 0x12
    params[1] = 5
    params[2] = 255
    params[3] = 0
    params[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1)
    params[5] = 30  # Report rate (30Hz)
    params[6] = 1
    params[7] = 3
    params[8] = 5
    params[9] = Cmd_ReportTag & 0xff
    params[10] = (Cmd_ReportTag >> 8) & 0xff
    cmd_pack_and_tx(ser, params)
    time.sleep(0.2)
    
    # Wake up sensor
    cmd_pack_and_tx(ser, [0x03])
    time.sleep(0.2)
    
    # Enable automatic reporting
    cmd_pack_and_tx(ser, [0x19])
    time.sleep(0.2)
    print("IMU configuration sent.")

def map_yaw_to_servo(yaw_deg_now, pitch_deg_now):
    """【修改】把(当前-初始)的IMU角度 映射为 (舵机初始位置 + 变化量)"""
    if imu_state.euler_init is None:
        # 还没首帧，就按当前角当作0
        yaw0, pitch0 = yaw_deg_now, pitch_deg_now
    else:
        yaw0, pitch0 = imu_state.euler_init[2], imu_state.euler_init[1]

    # 1. 计算IMU的角度变化量
    yaw_rel   = yaw_deg_now   - yaw0
    pitch_rel = pitch_deg_now - pitch0

    # 2. 将角度变化量 转换为 舵机位置的变化量
    relative_pos_z = int(round(yaw_rel   * UNITS_PER_DEG * MULTI_TURN_GAIN))
    relative_pos_x = int(round(pitch_rel * UNITS_PER_DEG * MULTI_TURN_GAIN))

    # 3. 最终目标位置 = 舵机初始位置 + 舵机位置变化量
    pos_z = imu_state.servo_init_pos_z + relative_pos_z
    pos_x = imu_state.servo_init_pos_x + relative_pos_x

    # 扩展范围限幅
    pos_z = max(MIN_POSITION_VALUE_extend, min(MAX_POSITION_VALUE_extend, pos_z))
    pos_x = max(MIN_POSITION_VALUE_extend, min(MAX_POSITION_VALUE_extend, pos_x))
    return pos_z, pos_x


def initialize_servos():
    portHandler = PortHandler(SERVO_PORT)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

    if not portHandler.openPort():
        print("Failed to open the servo port"); quit()
    if not portHandler.setBaudRate(SERVO_BAUDRATE):
        print("Failed to change the servo baudrate"); quit()

    # --- 切换到扩展位置模式 (4) 并使能扭矩 ---
    for dxl_id in [DXL_ID_Z, DXL_ID_X]:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    
    print("Servos initialized, switched to Extended Position Mode, torque enabled.")

    # --- 【新增】读取舵机当前位置作为初始值 ---
    print("Reading initial servo positions...")

    def read_servo_position(dxl_id):
        # 读取4字节的当前位置
        pos_unsigned, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] {packetHandler.getTxRxResult(dxl_comm_result)}")
            return 0
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] {packetHandler.getRxPacketError(dxl_error)}")
            return 0
        
        # 将读取到的 32位无符号数 转换为 32位有符号数
        if pos_unsigned > 2147483647: # (2^31 - 1)
            pos_signed = pos_unsigned - 4294967296 # (2^32)
        else:
            pos_signed = pos_unsigned
        
        return pos_signed

    # 读取并保存舵机Z的初始位置
    imu_state.servo_init_pos_z = read_servo_position(DXL_ID_Z)
    print(f"[ID:{DXL_ID_Z}] Initial Position (Z): {imu_state.servo_init_pos_z}")
    
    # 读取并保存舵机X的初始位置
    imu_state.servo_init_pos_x = read_servo_position(DXL_ID_X)
    print(f"[ID:{DXL_ID_X}] Initial Position (X): {imu_state.servo_init_pos_x}")
    # --- 【新增结束】 ---

    return portHandler, packetHandler, groupBulkWrite


def control_servos(groupBulkWrite, pos_z, pos_x):
    """
    使用 GroupBulkWrite 向舵机 (1 和 2) 发送目标位置
    """
    LEN_GOAL_POSITION = 4 # 目标位置的数据长度为 4 字节

    # 1. 清空参数
    groupBulkWrite.clearParam()

    # 2. 添加舵机 1 (Z, Yaw) 的目标位置参数
    #    将整数位置转换为4字节列表
    data_z = [DXL_LOBYTE(DXL_LOWORD(pos_z)), DXL_HIBYTE(DXL_LOWORD(pos_z)),
              DXL_LOBYTE(DXL_HIWORD(pos_z)), DXL_HIBYTE(DXL_HIWORD(pos_z))]
    
    addparam_result = groupBulkWrite.addParam(DXL_ID_Z, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, data_z)
    if not addparam_result:
        print(f"[ID:{DXL_ID_Z}] groupBulkWrite.addParam failed")
        return

    # 3. 添加舵机 2 (X, Pitch) 的目标位置参数
    data_x = [DXL_LOBYTE(DXL_LOWORD(pos_x)), DXL_HIBYTE(DXL_LOWORD(pos_x)),
              DXL_LOBYTE(DXL_HIWORD(pos_x)), DXL_HIBYTE(DXL_HIWORD(pos_x))]
              
    addparam_result = groupBulkWrite.addParam(DXL_ID_X, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, data_x)
    if not addparam_result:
        print(f"[ID:{DXL_ID_X}] groupBulkWrite.addParam failed")
        return

    # 4. 发送批量写入数据包
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        # 无法访问 packetHandler，仅打印错误码
        print(f"groupBulkWrite.txPacket failed: {dxl_comm_result}")




def main():


    # --- IMU Initialization ---
    try:
        imu_ser = serial.Serial(IMU_PORT, IMU_BAUDRATE, timeout=2)
    except serial.SerialException as e:
        print(f"Error opening IMU serial port {IMU_PORT}: {e}")
        return

    # --- Servo Initialization ---
    # 【修改】接收 groupBulkWrite
    servo_port_handler, servo_packet_handler, groupBulkWrite = initialize_servos()

    # --- Configure IMU and get initial reading ---
    configure_imu(imu_ser)
    
    print("Reading initial IMU data to set servo positions...")
    start_time = time.time()
    while not imu_state.is_initialized:
        if imu_ser.in_waiting > 0:
            byte = imu_ser.read(1)
            if byte:
                cmd_get_pkt(byte[0])
        if time.time() - start_time > 5: # 5-second timeout
            print("Failed to initialize IMU within the timeout period.")
            imu_ser.close()
            servo_port_handler.closePort()
            return

    # --- Main Loop: Continuously read IMU data ---
    print("\nStarting continuous data reading and servo control loop. Press Ctrl+C to exit.")
    try: 
        while True: 
            if imu_ser.in_waiting > 0: 
# Read all available bytes to process them faster 
                data_bytes = imu_ser.read(imu_ser.in_waiting) 
                for byte in data_bytes: 
                    cmd_get_pkt(byte) 
                    # 【修改】主循环中增加舵机控制 
            if imu_state.is_initialized: 
                # 1. 获取当前欧拉角 
                current_pitch = imu_state.current_euler[1] 
                current_yaw = imu_state.current_euler[2] 
                # 2. 将角度映射为舵机位置 
                pos_z, pos_x = map_yaw_to_servo(current_yaw, current_pitch) 
                # 3. 发送控制指令 
                control_servos(groupBulkWrite, pos_z, pos_x) 
                # 4. 打印实时状态 
                print(f"Yaw:{current_yaw:.2f} -> PosZ:{pos_z} | Pitch:{current_pitch:.2f} -> PosX:{pos_x} ", end='\r') 
            else: 
                # IMU 尚未初始化（理论上应该已经完成） 
                print("Waiting for IMU initialization...", end='\r')
                time.sleep(0.01) 
                # 循环延时，避免CPU占用过高 
    except KeyboardInterrupt: 
        print("\nExiting program.") 
    finally: 
# --- Cleanup --- 
# 【新增】清理 groupBulkWrite 
        groupBulkWrite.clearParam() # Disable servo torque 
        for dxl_id in [DXL_ID_Z, DXL_ID_X]: 
            servo_packet_handler.write1ByteTxRx(servo_port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE) # Close ports 
            imu_ser.close() 
            servo_port_handler.closePort() 
            print("Ports closed and servos disabled.")


if __name__ == "__main__":
    main()