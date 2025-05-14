#!/usr/bin/python3
import serial
import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from hjarm_engineer_driver.msg import ControlInfo, refereeKeyboard
import threading

car_joint_state = JointState()
car_joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称
car_joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # 初始化关节位置
car_joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # 初始化关节速度
car_joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]         # 初始化关节力矩

custom_mode = 0
customing = 1
INITJOINTPOS = [1.7, 0, -0.4, -2.95, 0, 0]
target_positions = INITJOINTPOS
joint4_offset = 0
## rad from -6.2 to 6.2
def dm_data_to_rad(data):
    x_min = -12.5
    x_max = 12.5
    bits = 16
    span = x_max - x_min
    offset = x_min
    return data * span / ((1 << bits) - 1) + offset


def dji_data_to_rad(data):
    return data / 8192.0 * 6.5

def convert_cc_to_car(joint_values):
    """
    Convert the joint values from the custom controller to the car's joint values
    """
    global joint4_offset

    
    car_joint_values = joint_values[:]
    if joint4_offset == 0 or not custom_mode:
        joint4_offset = car_joint_values[4]
    car_joint_values[0] = joint_values[0]
    car_joint_values[0] += 1.58
    if car_joint_values[0] > 3.1:
        car_joint_values[0] -= 3.1
    elif car_joint_values[0] < -3.1:
        car_joint_values[0] += 3.1
    
    car_joint_values[1] = -joint_values[1]
    car_joint_values[1] /= 4

    car_joint_values[2] = -joint_values[2]
    car_joint_values[2] -= 2.14
    car_joint_values[2] /= 2
    
    # car_joint_values[4] -= joint4_offset
    
    car_joint_values[5] -= 3.1
    
    return car_joint_values

def motor_position_callback(data):
    """
    订阅电机位置话题后的回调函数
    """
    global car_joint_state, custom_mode
    car_joint_state.position[0] = data.joint1_angle
    car_joint_state.position[1] = data.joint2_angle
    car_joint_state.position[2] = data.joint3_angle
    car_joint_state.position[3] = data.joint4_angle
    car_joint_state.position[4] = data.joint5_angle
    car_joint_state.position[5] = data.joint6_angle
    custom_mode = data.mode

def joint_pub(jointstate):
    state = JointState()
    state.header.stamp = rospy.Time.now()
    state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    state.position = jointstate
    joint_set_states_pub.publish(state)
 

def process_jointset(frame_data):
    if not customing:
        return
    global target_positions
    
    if not frame_data or len(frame_data) < 19:  # 确保至少有帧头(7字节)+6个关节数据(12字节)
        return
    
    # 检查帧头是否符合A5 ** ** ** ** 02 03模式
    if not (frame_data[0] == 0xA5 and frame_data[5] == 0x02 and frame_data[6] == 0x03):
        return
    
    # 解析六个关节的角度值
    joint_values = []
    for i in range(6):
        # 每个关节用2字节表示，从第8个字节(索引7)开始
        offset = 7 + i * 2
        # 小端序解析，低字节在前，高字节在后
        angle = frame_data[offset] | (frame_data[offset+1] << 8)
        joint_values.append(angle)
    
    # 转换角度值为目标位置弧度
    for i in range(6):
        if i <= 3:
            target_positions[i] = dm_data_to_rad(joint_values[i])
        else:
            target_positions[i] = dji_data_to_rad(joint_values[i])
    
    # 将目标位置转换为车的关节值
    target_positions = convert_cc_to_car(target_positions)
    joint_pub(target_positions)

def process_keyboard(frame_data):
    """处理一个完整的键盘数据帧并打印键盘事件"""
    global pressed_keys, target_positions, customing
    if not frame_data or len(frame_data) < 19:  # 确保至少有帧头(7字节)+键盘数据(偏移量8+长度2=10)
        return
    
    # 检查帧头是否符合A5 ** ** ** ** 04 03模式
    if not (frame_data[0] == 0xA5 and frame_data[5] == 0x04 and frame_data[6] == 0x03):
        return
    
    # 提取键盘按键信息（偏移量8，长度2字节）
    keyboard_offset = 7 + 8  # 帧头7字节 + 偏移量8
    keyboard_data = frame_data[keyboard_offset] | (frame_data[keyboard_offset + 1] << 8)
    
    # 键盘按键映射
    key_map = {
        0: 'W',
        1: 'S',
        2: 'A',
        3: 'D',
        4: 'Shift',
        5: 'Ctrl',
        6: 'Q',
        7: 'E',
        8: 'R',
        9: 'F',
        10: 'G',
        11: 'Z',
        12: 'X',
        13: 'C',
        14: 'V',
        15: 'B'
    }
    
    # 检查哪些键被按下并打印
    pressed_keys = []
    for bit, key in key_map.items():
        if keyboard_data & (1 << bit):
            pressed_keys.append(key)

    if 'G' in pressed_keys and 'Ctrl' in pressed_keys:
        customing = 0
        target_positions = [1.93, -0.97, -0.82, -2.0, 0, 0]
    elif 'G' in pressed_keys:
        customing = 0
        target_positions = [1.7, -2.15, -1.13, -2.95, 0, 0]

    if 'R' in pressed_keys and 'Ctrl' in pressed_keys:
        threading.Thread(store_task).start()

    if 'F' in pressed_keys:
        # back to custom controller mode
        target_positions = INITJOINTPOS
        customing = 1

    if pressed_keys:
        print(f"按下的键: {', '.join(pressed_keys)}")


def process_referee_data(ser):
    buffer = bytearray()
    frame_start_idx = -1
    frame_type = None  # 用于标记当前帧的类型：'joint' 或 'keyboard'
    
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)
                # 寻找并处理数据帧
                i = 0
                while i < len(buffer):
                    # 检查是否有足够的字节判断模式 (至少需要7个字节)
                    if i + 6 < len(buffer):
                        # 检查是否匹配关节设置模式: A5 ** ** ** ** 02 03
                        if (buffer[i] == 0xA5 and 
                            buffer[i+5] == 0x02 and 
                            buffer[i+6] == 0x03):
                            
                            # 找到新的匹配帧头
                            if frame_start_idx >= 0:
                                # 如果已经找到了上一个帧头，提取完整帧
                                frame_data = buffer[frame_start_idx:i]
                                if frame_type == 'joint':
                                    process_jointset(frame_data)
                                elif frame_type == 'keyboard':
                                    process_keyboard(frame_data)
                            
                            # 更新帧开始位置和类型
                            frame_start_idx = i
                            frame_type = 'joint'
                            i += 7  # 跳过已检查的模式字节
                            continue

                        # 检查是否匹配键盘模式: A5 ** ** ** ** 04 03
                        elif (buffer[i] == 0xA5 and 
                            buffer[i+5] == 0x04 and 
                            buffer[i+6] == 0x03):
                            
                            # 找到新的匹配帧头
                            if frame_start_idx >= 0:
                                # 如果已经找到了上一个帧头，提取完整帧
                                frame_data = buffer[frame_start_idx:i]
                                if frame_type == 'joint':
                                    process_jointset(frame_data)
                                elif frame_type == 'keyboard':
                                    process_keyboard(frame_data)
                            
                            # 更新帧开始位置和类型
                            frame_start_idx = i
                            frame_type = 'keyboard'
                            i += 7  # 跳过已检查的模式字节
                            continue
                        
                        # 检查是否匹配其他格式: A5 ** ** ** ** 04 03 (保留原有功能)
                        elif (buffer[i] == 0xA5 and 
                            buffer[i+5] == 0x04 and 
                            buffer[i+6] == 0x03):
                            
                            # 找到新的匹配帧头
                            if frame_start_idx >= 0:
                                # 如果已经找到了上一个帧头，提取完整帧
                                frame_data = buffer[frame_start_idx:i]
                                if frame_type == 'joint':
                                    process_jointset(frame_data)
                                elif frame_type == 'keyboard':
                                    process_keyboard(frame_data)
                            
                            # 更新帧开始位置和类型 (这里暂时当作关节数据处理)
                            frame_start_idx = i
                            frame_type = 'joint'
                            i += 7  # 跳过已检查的模式字节
                            continue
                        
                    # 如果有设置帧开始位置，检查是否找到下一个A5作为结束
                    if frame_start_idx >= 0 and i > frame_start_idx and buffer[i] == 0xA5:
                        # 找到A5作为结束标记
                        frame_data = buffer[frame_start_idx:i]
                        if frame_type == 'joint':
                            process_jointset(frame_data)
                        elif frame_type == 'keyboard':
                            process_keyboard(frame_data)
                        
                        # 不增加索引，让下一次循环检查这个A5是否为新帧的开始
                        frame_start_idx = -1
                        frame_type = None
                        continue
                    
                    i += 1
                
                # 处理缓冲区，保留最后可能的不完整帧
                if frame_start_idx >= 0:
                    buffer = buffer[frame_start_idx:]
                    frame_start_idx = 0
                else:
                    # 如果没有正在处理的帧，只保留最后一个字节(可能是下一个A5)
                    buffer = buffer[-1:] if buffer else bytearray()
            else:
                rospy.sleep(0.01)
        except OSError:
            print("dev os error")
            continue


def set_arm_pump(pump_i, state):
    puber = rospy.Publisher('/keyboardInfo', refereeKeyboard, queue_size=10)
    key_info = refereeKeyboard()
    if (state == 1):
        key_info.keyCtrl = True
        if pump_i == 0:
            key_info.keyZ = True
        elif pump_i == 1:
            key_info.keyX = True
        elif pump_i == 2:
            key_info.keyC = True

    puber.publish(key_info)

def check_done(current_position, target_position):
    done_flag = 1
    for joint_i in range(6):
        if current_position[joint_i] - target_position[joint_i] > 0.01:
            done_flag = 0
    return done_flag

def store_task():
    global customing, target_position
    if not customing:
        return
    customing = 0
    target_positions = [1.7, -2.15, -1.13, -2.95, 0, 0] # TODO: check position
    while not check_done(car_joint_state.position, target_position):
        rospy.sleep(0.1)
    set_arm_pump(1, 1) # open store pump
    set_arm_pump(2, 0) # close arm pump
    store_time = rospy.Time.now()
    while rospy.Time.now() - store_time < 0.5:
        rospy.sleep(0.1)
    set_arm_pump(0, keyboard_pub)
    customing = 1

def get_golden_task():
    global customing, target_position
    if not customing:
        return
    customing = 0
    target_positions = [1.7, -2.15, -1.13, -2.95, 0, 0] # TODO: check position
    while not check_done(car_joint_state.position, target_position):
        rospy.sleep(0.1)
    
    set_arm_pump(1, 1) # open arm pump
    set_arm_pump(2, 0) # close gloden pump
    store_time = rospy.Time.now()
    while rospy.Time.now() - store_time < 0.5:
        rospy.sleep(0.1)
    set_arm_pump(0, keyboard_pub)
    customing = 1

if __name__ == '__main__':
    rospy.init_node('cc_controller', anonymous=True)
    rate = rospy.Rate(10)
    joint_set_states_pub = rospy.Publisher('/joint_set_states', JointState, queue_size=10)
    keyboard_pub = rospy.Publisher('/keyboardInfo', refereeKeyboard, queue_size=10)
    rospy.Subscriber('/controlInfo', ControlInfo, motor_position_callback)



    serial_port = '/dev/ttyUSB_ttl'
    baudrate = 921600
    # baudrate = 115200

    # 打开串口
    try:
        ser = serial.Serial(serial_port, baudrate, timeout=0.1)
        print(f"成功打开串口 {serial_port}，波特率: {baudrate}")
    except Exception as e:
        print(f"打开串口失败: {e}")
        import sys
        sys.exit(1)

    # threading.Thread(target=process_referee_data, args=(ser,)).start()
    process_referee_data(ser)
