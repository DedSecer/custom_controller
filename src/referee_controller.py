#! /usr/bin/python
import serial
import struct
import time
import rospy
from sensor_msgs.msg import JointState
from hjarm_engineer_driver.msg import ControlInfo
import threading

car_joint_state = JointState()
car_joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称
car_joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # 初始化关节位置
car_joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # 初始化关节速度
car_joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]         # 初始化关节力矩

target_positions = [0, 0, 0, 0, 0, 0]
next_positions = [0, 0, 0, 0, 0, 0]

# rc r switch is up, car_mode = 2
# rc r switch is mid, car_mode = 1
# else, car_mode = 0
car_mode = 0

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
    global next_positions
    step_time = 1.0 / rate
    car_current_joint_state = car_joint_state.position[joint_index]
    while not rospy.is_shutdown():
        # keep update pose before auto mode
        if car_mode != 2:
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
    global joint4_offset

    
    car_joint_values = joint_values[:]
    if joint4_offset == 0 or car_mode != 2:
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
    
    # if car_joint_values[3] > 3.1:
    #     car_joint_values[3] -= 3.1
    # elif car_joint_values[3] < -3.1:
    #     car_joint_values[3] += 3.1
    
    car_joint_values[4] -= joint4_offset
    
    car_joint_values[5] -= 3.1
    
    return car_joint_values


def read_serial_data(ser):
    """
    从串口读取数据，解析帧格式为A5 ** ** ** ** 02 03开头的数据
    """
    global target_positions
    
    buffer = bytearray()
    frame_start_idx = -1
    
    while not rospy.is_shutdown():
        # 读取可用数据
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)
            
            # 寻找并处理数据帧
            i = 0
            while i < len(buffer):
                # 检查是否有足够的字节判断模式 (至少需要7个字节)
                if i + 6 < len(buffer):
                    # 检查是否匹配模式: A5 ** ** ** ** 02 03
                    if (buffer[i] == 0xA5 and 
                        buffer[i+5] == 0x02 and 
                        buffer[i+6] == 0x03):
                        
                        # 找到新的匹配帧头
                        if frame_start_idx >= 0:
                            # 如果已经找到了上一个帧头，提取完整帧
                            frame_data = buffer[frame_start_idx:i]
                            process_frame(frame_data)
                        
                        # 更新帧开始位置
                        frame_start_idx = i
                        i += 7  # 跳过已检查的模式字节
                        continue
                    
                # 如果有设置帧开始位置，检查是否找到下一个A5作为结束
                if frame_start_idx >= 0 and i > frame_start_idx and buffer[i] == 0xA5:
                    # 找到A5作为结束标记
                    frame_data = buffer[frame_start_idx:i]
                    process_frame(frame_data)
                    
                    # 不增加索引，让下一次循环检查这个A5是否为新帧的开始
                    frame_start_idx = -1
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


def process_frame(frame_data):
    """处理一个完整的数据帧并更新目标位置"""
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
    print(target_positions)

if __name__ == '__main__':
    rospy.init_node('custom_contoller_node', anonymous=True)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/joint_set_states', JointState, queue_size=10)
    rospy.Subscriber('/controlInfo', ControlInfo, motor_position_callback)

    # 使用固定的COM3串口和921600波特率
    serial_port = '/dev/ttyUSB0'
    # baudrate = 921600
    baudrate = 115200

    
    # 打开串口
    try:
        ser = serial.Serial(serial_port, baudrate, timeout=0.1)
        print(f"成功打开串口 {serial_port}，波特率: {baudrate}")
    except Exception as e:
        print(f"打开串口失败: {e}")
        import sys
        sys.exit(1)

    # 启动关节控制线程
    threading.Thread(target=move_joint_with_velocity_limit, args=(0, 0.4, 1000)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(1, 0.4, 1000)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(2, 0.4, 1000)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(3, 1, 1000)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(4, 8, 400)).start()
    threading.Thread(target=move_joint_with_velocity_limit, args=(5, 4, 400)).start()

    # 启动发布关节状态的线程
    threading.Thread(target=publish_step_job, args=(pub, 1000)).start()
    
    # 启动串口读取线程
    threading.Thread(target=read_serial_data, args=(ser,)).start()
    
    # 主循环，保持程序运行
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n关闭串口...")
        ser.close()
        print("程序已停止")