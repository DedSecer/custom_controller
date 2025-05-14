#! /usr/bin/python
import serial
import struct
import time
import rospy
from sensor_msgs.msg import JointState
from hjarm_engineer_driver.msg import ControlInfo, refereeKeyboard
import threading

car_joint_state = JointState()
car_joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称
car_joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # 初始化关节位置
car_joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # 初始化关节速度
car_joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]         # 初始化关节力矩

INITJOINTPOS = [1.7, 0, -0.4, -2.95, 0, 0]
target_positions = INITJOINTPOS
next_positions = [0, 0, 0, 0, 0, 0]

# rc r switch is up, car_mode = 2
# rc r switch is mid, car_mode = 1
# else, car_mode = 0
car_mode = 0

jointset_available = 1

joint4_offset = 0

pressed_keys = []

move_path_running = 0

done_flag = 0

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


def publish_joint_state(pub, joint_values):
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_state.position = joint_values
    pub.publish(joint_state)


def move_joint_with_velocity_limit(joint_index, max_vel, rate):
    """
    分段执行，从 current_positions 到 target_positions，速度不超过 max_vel
    """
    global next_positions, jointset_available
    step_time = 1.0 / rate
    car_current_joint_state = car_joint_state.position[joint_index]
    while not rospy.is_shutdown():
        # keep update pose before auto mode
        if car_mode != 1:
            car_current_joint_state = car_joint_state.position[joint_index]
            rospy.sleep(step_time)
            continue

        global target_positions
        diff = target_positions[joint_index] - car_current_joint_state
        step = max_vel * step_time
        if abs(diff) > step:
            # wait for each joint to reach the target position
            car_current_joint_state += step if diff > 0 else -step
        next_positions[joint_index] = car_current_joint_state

        rospy.sleep(step_time)


def publish_step_job(pub, rate):
    global next_positions
    step_time = 1.0 / rate
    while not rospy.is_shutdown():
        publish_joint_state(pub, next_positions)
        rospy.sleep(step_time)

def publish_keyboard_job(pub, rate):
    global pressed_keys
    step_time = 1.0 / rate
    while not rospy.is_shutdown():
        key_info = refereeKeyboard()
        if 'W' in pressed_keys:
            key_info.keyW = True
        if 'A' in pressed_keys:
            key_info.keyA = True
        if 'S' in pressed_keys:
            key_info.keyS = True
        if 'D' in pressed_keys:
            key_info.keyD = True
        if 'Z' in pressed_keys:
            key_info.keyZ = True
        if 'X' in pressed_keys:
            key_info.keyX = True
        if 'C' in pressed_keys:
            key_info.keyC = True
        if 'Ctrl' in pressed_keys:
            key_info.keyCtrl = True
        if 'Shift' in pressed_keys:
            key_info.keyShift = True
        
        pub.publish(key_info)
        rospy.sleep(step_time)

def motor_position_callback(data):
    """
    订阅电机位置话题后的回调函数
    """
    global car_joint_state, car_mode
    car_joint_state.position[0] = data.joint1_angle
    car_joint_state.position[1] = data.joint2_angle
    car_joint_state.position[2] = data.joint3_angle
    car_joint_state.position[3] = data.joint4_angle
    car_joint_state.position[4] = data.joint5_angle
    car_joint_state.position[5] = data.joint6_angle
    car_mode = data.mode
    # rospy.loginfo(f"当前电机位置: {joint_state.position[2]} \n")
    

def convert_cc_to_car(joint_values):
    """
    Convert the joint values from the custom controller to the car's joint values
    """
    # global joint4_offset

    
    car_joint_values = joint_values[:]
    # if joint4_offset == 0 or car_mode != 1:
    #     joint4_offset = car_joint_values[4]
    car_joint_values[0] = joint_values[0]
    car_joint_values[0] += 2.1
    if car_joint_values[0] > 3.1:
        car_joint_values[0] -= 3.1
    elif car_joint_values[0] < -3.1:
        car_joint_values[0] += 3.1
    
    
    car_joint_values[1] = -joint_values[1]
    car_joint_values[1] /= 4

    car_joint_values[2] = -joint_values[2]
    car_joint_values[2] -= 2.14
    car_joint_values[2] /= 2
    
    # if car_joint_values[3] > 3.1:
    #     car_joint_values[3] -= 3.1
    # elif car_joint_values[3] < -3.1:
    #     car_joint_values[3] += 3.1
    
    # car_joint_values[4] -= joint4_offset
    
    car_joint_values[5] -= 3.1
    
    return car_joint_values


def read_serial_data(ser):
    """
    从串口读取数据，解析帧格式为A5 ** ** ** ** 02 03或A5 ** ** ** ** 03 04开头的数据
    """
    global target_positions
    buffer = bytearray()
    frame_start_idx = -1
    frame_type = None  # 用于标记当前帧的类型：'joint' 或 'keyboard'
    
    while not rospy.is_shutdown():


        # 读取可用数据
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


def process_jointset(frame_data):
    """处理一个完整的数据帧并更新目标位置"""
    global jointset_available
    if not jointset_available:
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

def process_keyboard(frame_data):
    """处理一个完整的键盘数据帧并打印键盘事件"""
    global pressed_keys, jointset_available, target_positions
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
        jointset_available = 0
        p = [[3.68, -0.15, -0.3, 0.23, 0.07, -0.016], [3.85, -1.37, -1.47, 0.01, 2.4, -0.24]]
        threading.Thread(target=move_path, args=(p,)).start()

    elif 'G' in pressed_keys:
        jointset_available = 0
        target_positions = [1.7, -2.15, -1.13, 0, 0, 0]

    if 'R' in pressed_keys and 'Ctrl' in pressed_keys:
        threading.Thread(store_task).start()

    if 'F' in pressed_keys and not move_path_running:
        # back to custom controller mode
        target_positions = INITJOINTPOS
        jointset_available = 1

    if pressed_keys:
        print(f"按下的键: {', '.join(pressed_keys)}")


def check_done():
    global done_flag, target_positions
    while not rospy.is_shutdown():
        done_flag_tmp = 1
        for joint_i in range(4):
            if abs(car_joint_state.position[joint_i] - target_positions[joint_i]) > 0.12: # or timeout
                done_flag_tmp = 0
                break
        done_flag = done_flag_tmp
        rospy.sleep(0.1)
        

def move_path(position_que):
    global target_positions, move_path_running, done_flag
    if move_path_running:
        return
    move_path_running = 1
    rate = 10
    for que_i in range(len(position_que)):
        print("que_i",que_i)
        target_positions = position_que[que_i]
        done_flag = 0
        while not done_flag:
            rospy.sleep(1.0/rate) 
    move_path_running = 0

def store_task():
    global storing, target_position
    if storing:
        return
    storing = 1
    target_positions = [1.7, -2.15, -1.13, -2.95, 0, 0] # TODO: check position
    while not done_flag:
        rospy.sleep(0.1)
    set_arm_pump(1, 1) # open store pump
    set_arm_pump(2, 0) # close arm pump
    store_time = rospy.Time.now()
    while rospy.Time.now() - store_time < 0.5:
        rospy.sleep(0.1)
    set_arm_pump(0, keyboard_pub)
    storing = 0

def get_golden_task():
    global storing, target_position
    if storing:
        return
    storing = 1
    target_positions = [1.7, -2.15, -1.13, -2.95, 0, 0] # TODO: check position
    while not done_flag:
        rospy.sleep(0.1)
    
    set_arm_pump(1, 1) # open arm pump
    set_arm_pump(2, 0) # close gloden pump
    store_time = rospy.Time.now()
    while rospy.Time.now() - store_time < 0.5:
        rospy.sleep(0.1)
    set_arm_pump(0, keyboard_pub)
    storing = 0

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



if __name__ == '__main__':
    rospy.init_node('custom_contoller_node', anonymous=True)
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

    # 启动关节控制线程
    threading.Thread(target=move_joint_with_velocity_limit, args=(0, 1.2, 2000)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(1, 1.2, 2000)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(2, 1.2, 2000)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(3, 2.3, 2000)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(4, 20, 800)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(5, 8, 800)).start()

    # 启动发布关节状态的线程
    threading.Thread(target=publish_step_job, args=(joint_set_states_pub, 1000)).start()
    # 启动发布键盘信息的线程
    threading.Thread(target=publish_keyboard_job, args=(keyboard_pub, 1000)).start()
    
    # 启动串口读取线程
    threading.Thread(target=read_serial_data, args=(ser,)).start()
    
    threading.Thread(target=check_done).start()

    
    # 主循环，保持程序运行
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n关闭串口...")
        ser.close()
        print("程序已停止")